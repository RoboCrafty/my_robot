# Simulated controller + web server + React UI in one container (used by render.yaml).
# Build from the repo root:  docker build -f deploy/Dockerfile -t parol6-sim .

# --- React UI ---
FROM node:22-slim AS ui
WORKDIR /ui
COPY rpi-controller/web/ui/package.json rpi-controller/web/ui/package-lock.json ./
RUN npm ci
COPY rpi-controller/web/ui/ ./
RUN npm run build

# --- Pinocchio (not packaged for Ubuntu 24.04) + C++ controller ---
FROM ubuntu:24.04 AS build
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ca-certificates python3 \
        libeigen3-dev libboost-filesystem-dev libboost-serialization-dev libboost-system-dev \
        liburdfdom-dev liburdfdom-headers-dev libconsole-bridge-dev libreadline-dev \
    && rm -rf /var/lib/apt/lists/*

ARG PINOCCHIO_VERSION=v4.1.0
# jrl-cmakemodules strips PROJECT_SOURCE_DIR from header paths with a global string(REPLACE), so the
# checkout path must not equal "/pinocchio" (or "/p"), or headers install flat/mangled.
RUN git clone --depth 1 --recurse-submodules --shallow-submodules --branch ${PINOCCHIO_VERSION} \
        https://github.com/stack-of-tasks/pinocchio /pin-src \
    && cmake -S /pin-src -B /pin-src/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/pinocchio \
        -DBUILD_PYTHON_INTERFACE=OFF -DGENERATE_PYTHON_STUBS=OFF -DBUILD_TESTING=OFF \
        -DBUILD_WITH_COLLISION_SUPPORT=OFF -DBUILD_UTILS=OFF \
    && cmake --build /pin-src/build --target install -j4 \
    && rm -rf /pin-src

COPY rpi-controller/CMakeLists.txt /app/rpi-controller/CMakeLists.txt
COPY rpi-controller/src /app/rpi-controller/src
WORKDIR /app/rpi-controller
RUN cmake -B build -DCMAKE_PREFIX_PATH=/opt/pinocchio && cmake --build build --target parolController -j4

# --- Runtime ---
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-venv \
        libboost-filesystem1.83.0 libboost-serialization1.83.0 libboost-system1.83.0 \
        liburdfdom-model4.0 liburdfdom-world4.0 liburdfdom-sensor4.0 liburdfdom-model-state4.0 \
        libconsole-bridge1.0 libreadline8t64 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /opt/pinocchio /opt/pinocchio
COPY --from=build /app/rpi-controller/build/parolController /app/rpi-controller/build/parolController
ENV LD_LIBRARY_PATH=/opt/pinocchio/lib
RUN ! ldd /app/rpi-controller/build/parolController | grep "not found"

RUN python3 -m venv /opt/venv
ENV PATH=/opt/venv/bin:$PATH
COPY rpi-controller/web/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

COPY rpi-controller/src/assets /app/rpi-controller/src/assets
COPY rpi-controller/web /app/rpi-controller/web
COPY --from=ui /ui/dist /app/rpi-controller/web/ui/dist
COPY --chmod=755 deploy/start.sh /app/start.sh

# The server rewrites poses.json when points or programs are saved.
RUN chown 1000:1000 /app/rpi-controller/web /app/rpi-controller/web/poses.json
USER 1000
# The host sets PORT (Render injects it); server.py falls back to 8000.
ENV PAROL_DEFAULT_UI=next
CMD ["/app/start.sh"]
