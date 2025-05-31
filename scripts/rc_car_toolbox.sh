#!/bin/bash

# === CONFIGURATION ===
YOCTO_SYSROOT="/opt/poky/4.0.26/sysroots/armv8a-poky-linux"
APP="./build/src/rc-car-nav"
PORT=2345

JETSON_IP="192.168.1.10"
JETSON_USER="root"
JETSON_TARGET_DIR="/home/root"
REMOTE_APP_PATH="${JETSON_TARGET_DIR}/rc-car-nav"

MODE="$1"

if [[ "$MODE" == "local" ]]; then
    echo "[*] Selected MODE: QEMU (local)"

    # Kill anything using the GDB port
    PID=$(lsof -ti tcp:$PORT)
    if [ -n "$PID" ]; then
        echo "[*] Port $PORT is in use (PID $PID). Killing..."
        kill -9 "$PID"
    fi

    # Start QEMU with GDB server in background
    echo "[*] Launching QEMU with GDB server on port $PORT..."
    qemu-aarch64 -d in_asm,cpu -L "$YOCTO_SYSROOT" -g $PORT "$APP" > qemu.log 2>&1 &
    QEMU_PID=$!

    # Wait for GDB port to open
    for i in {1..20}; do
        if nc -z localhost $PORT; then
            echo "[*] QEMU is ready on port $PORT."
            exit 0
        fi
        sleep 0.2
    done

    echo "[!] QEMU never opened port $PORT — killing PID $QEMU_PID"
    kill -9 $QEMU_PID
    exit 1

elif [[ "$MODE" == "remote" ]]; then
    echo "[*] Selected MODE: Jetson Nano (remote)"

    echo "[*] Building application..."
    make -j$(nproc)
    if [[ $? -ne 0 ]]; then
        echo "[!] Build failed"
        exit 1
    fi

    echo "[*] Uploading to Jetson..."
    scp "$APP" "${JETSON_USER}@${JETSON_IP}:${JETSON_TARGET_DIR}/"

    echo "[*] Killing any previous instances on Jetson..."
    ssh "${JETSON_USER}@${JETSON_IP}" "
        killall -q rc-car-nav gdbserver || true
    "

    echo "[*] Starting gdbserver on Jetson..."
    ssh "${JETSON_USER}@${JETSON_IP}" "
        nohup gdbserver :$PORT $REMOTE_APP_PATH >${JETSON_TARGET_DIR}/gdbserver.log 2>&1 &
    "

    echo "[*] GDB server launched on Jetson at ${JETSON_IP}:${PORT}"
    exit 0

else
    echo "Usage: $0 [local|remote]"
    exit 1
fi
