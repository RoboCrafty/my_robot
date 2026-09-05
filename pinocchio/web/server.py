#!/usr/bin/env python3
"""Web UI backend for the Parol6 arm.

Bridges the browser (WebSocket) to the C++ `parolController` (UDP text protocol).
Owns everything non-real-time: saved poses/sequences (persisted to poses.json)
and playback sequencing. The C++ side stays the real-time master.

Run:  python3 server.py            # then open http://<pi-ip>:8000
Deps: pip install fastapi "uvicorn[standard]"
"""
import asyncio
import json
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

HERE = Path(__file__).parent
CFG_FILE = HERE / "poses.json"
STATIC_DIR = HERE / "static"
ASSETS_DIR = HERE.parent / "src" / "assets"   # parol6.urdf + meshes/, shared with the C++ side
ESP_ADDR = ("127.0.0.1", 5005)   # where parolController listens (arg 2)

app = FastAPI()

clients: set[WebSocket] = set()
udp_transport: asyncio.DatagramTransport | None = None
play_task: asyncio.Task | None = None


def load_cfg() -> dict:
    if CFG_FILE.exists():
        try:
            return json.loads(CFG_FILE.read_text())
        except Exception:
            pass
    return {"poses": {}, "sequences": {}, "increments": [5, 10, 20]}


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
        try:
            state = json.loads(data.decode())
        except Exception:
            return
        asyncio.create_task(broadcast(json.dumps({"type": "state", **state})))


@app.on_event("startup")
async def _startup() -> None:
    global udp_transport
    loop = asyncio.get_running_loop()
    udp_transport, _ = await loop.create_datagram_endpoint(
        StateProto, remote_addr=ESP_ADDR
    )

    async def keepalive():
        # Registers our address with the controller so it keeps publishing state.
        while True:
            send_cmd("ping")
            await asyncio.sleep(1.0)

    asyncio.create_task(keepalive())


@app.get("/")
async def index():
    return FileResponse(STATIC_DIR / "index.html")


# The 3D viewer fetches the same URDF the controller runs on, so the model on
# screen can never drift from the model doing the kinematics.
app.mount("/assets", StaticFiles(directory=ASSETS_DIR), name="assets")
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


async def run_playback(steps: list[dict]) -> None:
    """steps: [{"angles": [6 floats], "dwell": seconds}]"""
    try:
        for st in steps:
            angles = st["angles"]
            send_cmd(" ".join(f"{x:.3f}" for x in angles))
            await asyncio.sleep(float(st.get("dwell", 1.0)))
        await broadcast(json.dumps({"type": "play_done"}))
    except asyncio.CancelledError:
        send_cmd("stop")
        raise


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    global play_task
    await ws.accept()
    clients.add(ws)
    await ws.send_text(json.dumps({"type": "config", **cfg}))
    try:
        while True:
            m = json.loads(await ws.receive_text())
            t = m.get("type")

            if t == "cmd":
                send_cmd(m["line"])

            elif t == "save_pose":
                cfg["poses"][m["name"]] = m["angles"]
                save_cfg(); await broadcast(json.dumps({"type": "config", **cfg}))

            elif t == "delete_pose":
                cfg["poses"].pop(m["name"], None)
                save_cfg(); await broadcast(json.dumps({"type": "config", **cfg}))

            elif t == "save_seq":
                cfg["sequences"][m["name"]] = m["steps"]
                save_cfg(); await broadcast(json.dumps({"type": "config", **cfg}))

            elif t == "delete_seq":
                cfg["sequences"].pop(m["name"], None)
                save_cfg(); await broadcast(json.dumps({"type": "config", **cfg}))

            elif t == "set_increments":
                cfg["increments"] = m["values"]
                save_cfg(); await broadcast(json.dumps({"type": "config", **cfg}))

            elif t == "play":
                if play_task and not play_task.done():
                    play_task.cancel()
                play_task = asyncio.create_task(run_playback(m["steps"]))

            elif t == "stop_play":
                if play_task and not play_task.done():
                    play_task.cancel()
                send_cmd("stop")

    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
