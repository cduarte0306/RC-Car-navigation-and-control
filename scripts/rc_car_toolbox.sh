#!/bin/bash

# === CONFIGURATION ===
YOCTO_SYSROOT="/opt/poky/4.0.26/sysroots/armv8a-poky-linux"
APP="./build/src/rc-car-nav"
PORT=2345

JETSON_IP="192.168.1.10"
JETSON_USER="root"
JETSON_TARGET_DIR="/opt/rc-car/rc-car-nav/"
REMOTE_APP_PATH="${JETSON_TARGET_DIR}/rc-car-nav"
GDBSERVER_PATH="/usr/bin/gdbserver"    # explicit path

MODE="$1"


if [[ "$MODE" == "local" ]]; then
    echo "[*] Selected MODE: QEMU (local)"
    # ... (your unchanged local branch) ...
    # [keep your existing QEMU + wait-for-port logic here]
    exit 0

elif [[ "$MODE" == "remote" ]]; then
    echo "[*] Selected MODE: Jetson Nano (remote)"

    echo "[*] Building host app..."
    cd build
    make -j"$(nproc)" && cd .. \
        || { echo "[!] Build failed"; exit 1; }

    echo "[*] Killing any previous gdbserver on Jetson..."
    ssh "${JETSON_USER}@${JETSON_IP}" \
        "${GDBSERVER_PATH} --version &>/dev/null && pkill -9 -f gdbserver || true"

    echo "[*] Uploading app to Jetson..."
    scp "$APP" "${JETSON_USER}@${JETSON_IP}:${JETSON_TARGET_DIR}/" \
        || { echo "[!] SCP failed"; exit 1; }

    echo "[*] Starting gdbserver on Jetson..."
    ssh "${JETSON_USER}@${JETSON_IP}" <<EOF
      nohup "${GDBSERVER_PATH}" :${PORT} "${REMOTE_APP_PATH}" \
        &> "${JETSON_TARGET_DIR}/updater-log.log" &
EOF

    # wait for port to open
    echo "[*] Waiting for gdbserver to listen on ${JETSON_IP}:${PORT}..."
    for i in {1..20}; do
      if nc -z "${JETSON_IP}" "${PORT}"; then
        echo "[*] gdbserver is up!"
        exit 0
      fi
      sleep 0.3
    done

    echo "[!] gdbserver never opened port ${PORT}"
    exit 1
    
elif [[ "$MODE" == "upload" ]]; then
    echo "[*] Selected MODE: Upload only"

    # ensure the built binary exists
    if [[ ! -f "$APP" ]]; then
        echo "[!] App not found: $APP"
        echo "[*] Try building first or pass path to existing binary."
        exit 1
    fi
    cd build
    make -j"$(nproc)" && cd .. \
        || { echo "[!] Build failed"; exit 1; }

    ssh "${JETSON_USER}@${JETSON_IP}" "killall -9 rc-car-updater || true"

    echo "[*] Uploading app to Jetson..."
    scp "$APP" "${JETSON_USER}@${JETSON_IP}:${JETSON_TARGET_DIR}/" \
        || { echo "[!] SCP failed"; exit 1; }

    echo "[*] Upload complete: ${REMOTE_APP_PATH}"
    exit 0

else
    echo "Usage: $0 [local|remote]"
    exit 1
fi
