#!/usr/bin/env python3
"""Web UI backend for the Parol6 arm.

Bridges the browser (WebSocket) to the C++ `parolController` (UDP text protocol).
Owns everything non-real-time: saved waypoints, programs (persisted to
poses.json) and the program interpreter. The C++ side stays the real-time master.

Run:  python3 server.py            # then open http://<pi-ip>:8000
Deps: pip install fastapi "uvicorn[standard]"
"""
import asyncio
import json
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

HERE = Path(__file__).parent
CFG_FILE = HERE / "poses.json"
STATIC_DIR = HERE / "static"
ASSETS_DIR = HERE.parent / "src" / "assets"   # URDF + meshes/, shared with the C++ side
ESP_ADDR = ("127.0.0.1", 5005)   # where parolController listens (arg 2)

NJ = 6
GRIP_MAX = 140
MAX_CALL_DEPTH = 16              # a program calling itself must not blow the stack

clients: set[WebSocket] = set()
udp_transport: asyncio.DatagramTransport | None = None
latest_state: dict = {}

EMPTY_CFG = {"poses": {}, "programs": {}, "increments": [5, 10, 20],
             "settings": {}, "grips": {}}


def normalise(raw: dict) -> dict:
    """Bring an on-disk config up to the current schema.

    Waypoints used to be bare joint arrays; they are now {angles, pose} so the UI
    can show where a point is without running IK. Programs used to be flat step
    lists under "sequences".
    """
    c = {**EMPTY_CFG, **raw}
    c["poses"] = {
        name: (p if isinstance(p, dict) else {"angles": list(p), "pose": None})
        for name, p in c.get("poses", {}).items()
    }
    for name, steps in raw.get("sequences", {}).items():
        if name in c["programs"]:
            continue
        c["programs"][name] = [
            {"id": f"m{i}", "type": "move", "motion": s.get("type", "j"),
             "wp": s.get("pose"), "dwell": float(s.get("dwell", 0) or 0)}
            for i, s in enumerate(steps) if s.get("pose")
        ]
    c.pop("sequences", None)
    return c


def load_cfg() -> dict:
    if CFG_FILE.exists():
        try:
            return normalise(json.loads(CFG_FILE.read_text()))
        except Exception:
            pass
    return normalise({})


def save_cfg() -> None:
    CFG_FILE.write_text(json.dumps(cfg, indent=2))


cfg = load_cfg()



def send_cmd(line: str) -> None:
    """Send one text command line to the C++ controller."""
    if udp_transport is not None:
        udp_transport.sendto(line.encode(), ESP_ADDR)


async def _safe_send(ws: WebSocket, msg: str) -> None:
    try:
        await ws.send_text(msg)
    except Exception:
        clients.discard(ws)


async def broadcast(msg: str) -> None:
    for ws in list(clients):
        await _safe_send(ws, msg)


class StateProto(asyncio.DatagramProtocol):
    """Receives JSON state datagrams from the C++ controller and fans them out."""

    def datagram_received(self, data: bytes, addr) -> None:
        global latest_state
        try:
            state = json.loads(data.decode())
        except Exception:
            return
        latest_state = state
        asyncio.create_task(broadcast(json.dumps({"type": "state", **state})))


@asynccontextmanager
async def lifespan(_: FastAPI):
    global udp_transport
    loop = asyncio.get_running_loop()
    udp_transport, _proto = await loop.create_datagram_endpoint(
        StateProto, remote_addr=ESP_ADDR
    )

    async def keepalive():
        # Registers our address with the controller so it keeps publishing state.
        while True:
            send_cmd("ping")
            await asyncio.sleep(1.0)

    task = asyncio.create_task(keepalive())
    try:
        yield
    finally:
        task.cancel()
        udp_transport.close()


app = FastAPI(lifespan=lifespan)


@app.get("/")
async def index():
    return FileResponse(STATIC_DIR / "index.html")


# The 3D viewer fetches the same URDF the controller runs on, so the model on
# screen can never drift from the model doing the kinematics.
app.mount("/assets", StaticFiles(directory=ASSETS_DIR), name="assets")
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


async def wait_for_move(timeout: float = 120.0) -> None:
    """Block until the controller reports the move finished.

    Moves must not be sent on a fixed timer: a slow segment would be truncated by
    the next command. The controller bumps `seq` when it accepts a motion command
    and publishes `busy` while one is in flight -- waiting for `seq` to change
    first is what distinguishes "not started yet" from "already finished", which
    a bare busy flag cannot express for a very short move.
    """
    loop = asyncio.get_running_loop()
    start = loop.time()
    seq0 = latest_state.get("seq")
    while loop.time() - start < 1.5 and latest_state.get("seq") == seq0:
        await asyncio.sleep(0.01)
    while latest_state.get("busy") and loop.time() - start < timeout:
        await asyncio.sleep(0.01)


def wp_angles(name: str) -> list[float] | None:
    """Joint angles of a taught waypoint, resolved at run time.

    Programs store the waypoint *name*, not a copy of its angles, so re-teaching
    a point updates every program that uses it.
    """
    p = cfg["poses"].get(name)
    if not p:
        return None
    a = p.get("angles") if isinstance(p, dict) else p
    return list(a) if a and len(a) == NJ else None


class ProgramRunner:
    """Walks a program node tree, one node at a time, against the controller.

    Nodes: move | gripper | wait | speed | loop | call | comment.
    Supports pause/resume and single-step; the currently executing node id is
    broadcast so the editor can highlight the running line.
    """

    def __init__(self, nodes: list[dict], step: bool = False):
        self.nodes = nodes
        self.node_id: str | None = None
        self.counters: dict[str, str] = {}
        self.error: str | None = None
        self.paused = step
        self.step = step
        # The final status has to be published from inside the task, while the
        # task is by definition still alive -- so "finished" is tracked here
        # rather than inferred from task.done().
        self.finished = False
        self._gate = asyncio.Event()
        if not step:
            self._gate.set()
        self.task: asyncio.Task | None = None

    def start(self) -> None:
        self.task = asyncio.create_task(self._main())

    # -- control ---------------------------------------------------------
    def pause(self) -> None:
        self.paused = True
        self._gate.clear()

    def resume(self) -> None:
        self.step = False
        self.paused = False
        self._gate.set()

    def step_once(self) -> None:
        self.step = True
        self.paused = False
        self._gate.set()

    def stop(self) -> None:
        self.finished = True
        self.node_id = None
        self.paused = False
        self.counters.clear()
        if self.task is not None:
            self.task.cancel()

    def status(self) -> dict:
        running = not self.finished and self.task is not None and not self.task.done()
        return {"type": "prog", "running": running, "paused": self.paused,
                "node": self.node_id, "counters": self.counters, "error": self.error}

    # -- execution -------------------------------------------------------
    async def _checkpoint(self, node_id: str) -> None:
        """Highlight a node, honouring pause/step before it runs."""
        self.node_id = node_id
        if self.step:
            self.pause()
        await push_prog()
        if not self._gate.is_set():
            await self._gate.wait()
            await push_prog()

    async def _main(self) -> None:
        try:
            await self._block(self.nodes, ())
        except asyncio.CancelledError:
            send_cmd("stop")
            raise
        except Exception as e:                      # a bad step must not kill the socket
            self.error = str(e)
            send_cmd("stop")
        self.finished = True
        self.node_id = None
        self.paused = False
        self.counters.clear()
        await push_prog()
        if not self.error:
            await broadcast(json.dumps({"type": "prog_done"}))

    async def _block(self, nodes: list[dict], stack: tuple[str, ...]) -> None:
        for node in nodes:
            await self._checkpoint(node.get("id", ""))
            await self._exec(node, stack)

    async def _exec(self, node: dict, stack: tuple[str, ...]) -> None:
        kind = node.get("type")

        if kind == "move":
            angles = wp_angles(node["wp"]) if node.get("wp") else node.get("angles")
            if not angles or len(angles) != NJ:
                raise ValueError(f"move: unknown waypoint {node.get('wp')!r}")
            joints = " ".join(f"{float(x):.3f}" for x in angles)
            send_cmd(f"movel q {joints}" if node.get("motion") == "l" else joints)
            await wait_for_move()
            if node.get("dwell"):
                await asyncio.sleep(float(node["dwell"]))

        elif kind == "gripper":
            v = max(0, min(GRIP_MAX, int(node.get("value", 0))))
            send_cmd(f"gripper {v}")
            # Open loop: the servo reports no position, so we can only wait.
            await asyncio.sleep(float(node.get("settle", 0.5)))

        elif kind == "wait":
            await asyncio.sleep(max(0.0, float(node.get("seconds", 0))))

        elif kind == "speed":
            send_cmd(f"speed {max(5, min(100, int(node.get('percent', 100))))}")

        elif kind == "loop":
            body = node.get("body") or []
            node_id = node.get("id", "")
            forever = node.get("mode") == "forever"
            total = max(1, int(node.get("count", 1)))
            i = 0
            while forever or i < total:
                i += 1
                self.counters[node_id] = f"{i}/∞" if forever else f"{i}/{total}"
                await push_prog()
                await self._block(body, stack)
                if not body:
                    await asyncio.sleep(0.05)   # keep an empty forever-loop cancellable
            self.counters.pop(node_id, None)

        elif kind == "call":
            name = node.get("name", "")
            if name in stack:
                raise ValueError(f"call: {name!r} calls itself")
            if len(stack) >= MAX_CALL_DEPTH:
                raise ValueError("call: nested too deeply")
            sub = cfg["programs"].get(name)
            if sub is None:
                raise ValueError(f"call: unknown program {name!r}")
            await self._block(sub, stack + (name,))

        elif kind == "comment":
            pass

        else:
            raise ValueError(f"unknown node type {kind!r}")


runner: ProgramRunner | None = None


async def push_prog() -> None:
    if runner is not None:
        await broadcast(json.dumps(runner.status()))



@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    global runner
    await ws.accept()
    clients.add(ws)
    await ws.send_text(json.dumps({"type": "config", **cfg}))
    if runner is not None:
        await ws.send_text(json.dumps(runner.status()))
    try:
        while True:
            m = json.loads(await ws.receive_text())
            t = m.get("type")

            if t == "cmd":
                send_cmd(m["line"])

            elif t == "save_pose":
                # Joints drive motion; the pose is the controller's own FK, kept
                # only so the UI can show where the waypoint is.
                cfg["poses"][m["name"]] = {"angles": m["angles"], "pose": m.get("pose")}
                await commit()

            elif t == "delete_pose":
                cfg["poses"].pop(m["name"], None)
                await commit()

            elif t == "rename_pose":
                old, new = m["name"], m["to"]
                if old in cfg["poses"] and new and new not in cfg["poses"]:
                    cfg["poses"][new] = cfg["poses"].pop(old)
                    # Programs reference waypoints by name, so retarget them too.
                    for nodes in cfg["programs"].values():
                        for n in walk(nodes):
                            if n.get("type") == "move" and n.get("wp") == old:
                                n["wp"] = new
                    await commit()

            elif t == "save_program":
                cfg["programs"][m["name"]] = m["nodes"]
                await commit()

            elif t == "delete_program":
                cfg["programs"].pop(m["name"], None)
                await commit()

            elif t == "set_increments":
                cfg["increments"] = m["values"]
                await commit()

            elif t == "save_settings":
                # UI preferences (jog mode, frame, speeds, step sizes).
                cfg["settings"] = m["values"]
                await commit()

            elif t == "save_grip":
                # Per-object grip: how far to close for that part, taught by hand.
                cfg["grips"][m["name"]] = {"open": int(m["open"]), "close": int(m["close"])}
                await commit()

            elif t == "delete_grip":
                cfg["grips"].pop(m["name"], None)
                await commit()

            elif t == "run":
                if runner is not None:
                    runner.stop()
                runner = ProgramRunner(m["nodes"], step=m.get("step", False))
                runner.start()
                await push_prog()

            elif t == "prog_ctl" and runner is not None:
                {"pause": runner.pause, "resume": runner.resume,
                 "step": runner.step_once, "stop": runner.stop}.get(m["action"], lambda: None)()
                if m["action"] == "stop":
                    send_cmd("stop")
                await push_prog()

    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)


def walk(nodes: list[dict]):
    """Yield every node in a program, descending into loop bodies."""
    for n in nodes:
        yield n
        if n.get("body"):
            yield from walk(n["body"])


async def commit() -> None:
    save_cfg()
    await broadcast(json.dumps({"type": "config", **cfg}))



if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
