#!/bin/bash

# === CONFIGURATION ===
YOCTO_SDK="/opt/poky/5.0.11"
YOCTO_SYSROOT="${YOCTO_SDK}/sysroots/armv8a-poky-linux"
CROSS_GDB="${YOCTO_SDK}/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-gdb"
APP="./build/src/rc-car-nav"
CORE_LOCAL="./rc-car-nav.core"
PORT=2345

JETSON_IP="192.168.1.10"
JETSON_USER="root"
JETSON_TARGET_DIR="/tmp/"
REMOTE_APP_PATH="${JETSON_TARGET_DIR}/rc-car-nav"
GDBSERVER_PATH="/usr/bin/gdbserver"    # explicit path

MODE="$1"


if [[ "$MODE" == "local" ]]; then
    echo "[*] Selected MODE: QEMU (local)"
    # ... (your unchanged local branch) ...
    # [keep your existing QEMU + wait-fbor-port logic here]
    exit 0

elif [[ "$MODE" == "remote" ]]; then
    echo "[*] Selected MODE: Jetson Nano (remote)"

    echo "[*] Building host app..."
    cd build
    make -j16 && cd .. \
        || { echo "[!] Build failed"; exit 0; }

    echo "[*] Killing any previous gdbserver on Jetson..."
    ssh "${JETSON_USER}@${JETSON_IP}" \
        "pkill -9 gdbserver || true; rm -f ${JETSON_TARGET_DIR}/gdbserver.log"

    echo "[*] Uploading app to Jetson..."
    scp "$APP" "${JETSON_USER}@${JETSON_IP}:${JETSON_TARGET_DIR}/" \
        || { echo "[!] SCP failed"; exit 0; }

    echo "[*] Starting gdbserver on Jetson..."
    ssh "${JETSON_USER}@${JETSON_IP}" \
        "nohup ${GDBSERVER_PATH} :${PORT} ${REMOTE_APP_PATH} > ${JETSON_TARGET_DIR}/gdbserver.log 2>&1 &"

    sleep 1  # Give gdbserver time to start and begin logging
    
    # wait for port to open
    echo "[*] Waiting for gdbserver to listen on ${JETSON_IP}:${PORT}..."
    for i in {1..20}; do
      if nc -z "${JETSON_IP}" "${PORT}"; then
        echo "[*] gdbserver is up!"
        echo "[*] Log file: ${JETSON_TARGET_DIR}/gdbserver.log"
        exit 0
      fi
      sleep 0.3
    done

    echo "[!] gdbserver never opened port ${PORT}"
    echo "[*] Checking remote log..."
    ssh "${JETSON_USER}@${JETSON_IP}" "cat ${JETSON_TARGET_DIR}/gdbserver.log" || true
    exit 0
    
elif [[ "$MODE" == "upload" ]]; then
    echo "[*] Selected MODE: Upload only"

    # ensure the built binary exists
    if [[ ! -f "$APP" ]]; then
        echo "[!] App not found: $APP"
        echo "[*] Try building first or pass path to existing binary."
        exit 0
    fi
    cd build
    make -j16 && cd .. \
        || { echo "[!] Build failed"; exit 0; }

    ssh "${JETSON_USER}@${JETSON_IP}" "killall -9 rc-car-updater || true"

    echo "[*] Uploading app to Jetson..."
    scp "$APP" "${JETSON_USER}@${JETSON_IP}:${JETSON_TARGET_DIR}/" \
        || { echo "[!] SCP failed"; exit 0; }

    echo "[*] Upload complete: ${REMOTE_APP_PATH}"
    exit 0

elif [[ "$MODE" == "coredump" ]]; then
    echo "[*] Selected MODE: Core Dump Debug"

    DEVICE_BIN_LOCAL="./rc-car-nav-device"
    DEVICE_LIBS_LOCAL="./device-libs"

    echo "[*] Extracting latest core dump from Jetson..."
    ssh "${JETSON_USER}@${JETSON_IP}" \
        "coredumpctl dump --output=/tmp/rc-car-nav.core" \
        || { echo "[!] coredumpctl failed — no core dump on device?"; exit 1; }

    echo "[*] Copying core dump and device binary to workstation..."
    scp "${JETSON_USER}@${JETSON_IP}:/tmp/rc-car-nav.core" "${CORE_LOCAL}" \
        || { echo "[!] SCP of core dump failed"; exit 1; }

    # Pull the exact binary that generated the core — may differ from local build if stripped
    scp "${JETSON_USER}@${JETSON_IP}:${REMOTE_APP_PATH}" "${DEVICE_BIN_LOCAL}" \
        || echo "[!] Could not copy device binary — symbol resolution may be incomplete"

    # Pull device-side shared libs that aren't in the SDK sysroot (NVIDIA/CUDA specific)
    echo "[*] Syncing device-specific libs (NVIDIA/CUDA) for symbol resolution..."
    mkdir -p "${DEVICE_LIBS_LOCAL}"
    rsync -az --ignore-errors \
        "${JETSON_USER}@${JETSON_IP}:/usr/lib/libnvinfer*" \
        "${JETSON_USER}@${JETSON_IP}:/usr/lib/libnvonnxparser*" \
        "${JETSON_USER}@${JETSON_IP}:/usr/lib/libcuda*" \
        "${JETSON_USER}@${JETSON_IP}:/usr/local/cuda-12.6/lib/libcudart*" \
        "${JETSON_USER}@${JETSON_IP}:/usr/lib/libopencv_core*" \
        "${JETSON_USER}@${JETSON_IP}:/usr/lib/libopencv_dnn*" \
        "${DEVICE_LIBS_LOCAL}/" 2>/dev/null || true

    echo "[*] Core dump saved to:   ${CORE_LOCAL}"
    echo "[*] Device binary saved:  ${DEVICE_BIN_LOCAL}"

    if [[ ! -f "$CROSS_GDB" ]]; then
        echo "[!] Cross-GDB not found at ${CROSS_GDB}"
        exit 1
    fi

    # Prefer device binary (exact match for core) but fall back to local build for symbols
    GDB_BIN="${APP}"
    if [[ -f "${DEVICE_BIN_LOCAL}" ]]; then
        echo "[*] Using device binary for address matching, local build for symbols"
        # Load device binary first, then add symbol file from local debug build
        EXTRA_SYM="-ex \"add-symbol-file ${APP}\""
        GDB_BIN="${DEVICE_BIN_LOCAL}"
    fi

    echo "[*] Launching cross-GDB..."
    echo "[*]   Binary:  ${GDB_BIN}"
    echo "[*]   Core:    ${CORE_LOCAL}"
    echo "[*]   Sysroot: ${YOCTO_SYSROOT}"

    "${CROSS_GDB}" \
        -ex "set sysroot ${YOCTO_SYSROOT}" \
        -ex "set solib-search-path ${YOCTO_SYSROOT}/usr/lib:${YOCTO_SYSROOT}/lib:${DEVICE_LIBS_LOCAL}" \
        -ex "set auto-load safe-path /" \
        -ex "set print thread-events off" \
        -ex "set pagination off" \
        -ex "info proc mappings" \
        -ex "thread 1" \
        -ex "x/30i 0x0000aaaad276dc50" \
        -ex "bt" \
        -ex "thread apply all bt" \
        "${GDB_BIN}" "${CORE_LOCAL}"

else
    echo "Usage: $0 [local|remote|upload|coredump]"
    exit 0
fi
