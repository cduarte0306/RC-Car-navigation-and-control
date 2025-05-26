#!/bin/bash

# === CONFIGURATION ===
NANO_IP="192.168.1.10"
TARGET_DIR="/home/root"
APP_NAME="rc-car-nav"
LOCAL_APP_PATH="build/src/${APP_NAME}"

# === EXECUTION ===
make -j16
echo "Uploading ${LOCAL_APP_PATH} to ${NANO_IP}:${TARGET_DIR}..."
scp "${LOCAL_APP_PATH}" root@${NANO_IP}:${TARGET_DIR}/

echo "Killing any previous ${APP_NAME} or gdbserver instances..."
ssh root@${NANO_IP} "
    killall rc-car-nav
    killall gdbserver
"

echo "Starting gdbserver on Nano..."
ssh root@${NANO_IP} "nohup gdbserver :2345 ${TARGET_DIR}/${APP_NAME} >${TARGET_DIR}/gdbserverlog 2>&1 &"

# === DONE ===
