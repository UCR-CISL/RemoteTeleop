#!/bin/bash
# start_connection.sh - Configure wired ROS connection to the Lubao delivery robot
# Usage: source start_connection.sh [interface]
#
# The Lubao protocol summary documents wired mode as:
#   laptop IP: 192.168.123.88
#   robot IP:  192.168.123.100
#   ROS master: http://192.168.123.100:11311

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "[WARN] This script should be sourced, not executed directly!"
    echo "       Run: source start_connection.sh [interface]"
    echo ""
    echo "Continuing anyway, but environment won't persist to your shell..."
    echo ""
fi

set -o pipefail

ROBOT_IP="192.168.123.100"
LAPTOP_IP="192.168.123.88"
NETMASK_CIDR="24"
ROS_MASTER="http://${ROBOT_IP}:11311"
HOST_ALIAS="qcs6490-odk"
WIRED_IFACE="${1:-${ROBOT_WIRED_IFACE:-eth0}}"

echo "=== Lubao Robot Wired Connection Setup ==="
echo "[INFO] Interface: ${WIRED_IFACE}"

# 0. Initialize conda/mamba.
echo "[INFO] Initializing conda..."
if [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/anaconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/miniforge3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniforge3/etc/profile.d/conda.sh"
elif [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
else
    echo "[ERROR] Cannot find conda installation"
    return 1 2>/dev/null || exit 1
fi

if [ -f "$HOME/anaconda3/etc/profile.d/mamba.sh" ]; then
    source "$HOME/anaconda3/etc/profile.d/mamba.sh"
elif [ -f "$HOME/miniforge3/etc/profile.d/mamba.sh" ]; then
    source "$HOME/miniforge3/etc/profile.d/mamba.sh"
fi

# 1. Verify interface exists.
if ! ip link show "$WIRED_IFACE" > /dev/null 2>&1; then
    echo "[ERROR] Interface '${WIRED_IFACE}' not found."
    echo "Available interfaces:"
    ip -br link 2>/dev/null || true
    echo ""
    echo "Run with the correct interface, for example:"
    echo "  source start_connection.sh enx0123456789ab"
    return 1 2>/dev/null || exit 1
fi

# 2. Configure wired IP address.
echo "[INFO] Configuring ${WIRED_IFACE} with ${LAPTOP_IP}/${NETMASK_CIDR}..."
if ! ip addr show dev "$WIRED_IFACE" | grep -q "inet ${LAPTOP_IP}/"; then
    sudo ip addr flush dev "$WIRED_IFACE" scope global
    sudo ip addr add "${LAPTOP_IP}/${NETMASK_CIDR}" dev "$WIRED_IFACE"
fi
sudo ip link set "$WIRED_IFACE" up

if ip addr show dev "$WIRED_IFACE" | grep -q "inet ${LAPTOP_IP}/"; then
    echo "[OK] ${WIRED_IFACE} has IP ${LAPTOP_IP}"
else
    echo "[ERROR] Failed to configure ${WIRED_IFACE} with ${LAPTOP_IP}"
    return 1 2>/dev/null || exit 1
fi

# 3. Ensure host alias points at the wired robot IP.
if grep -qE "[[:space:]]${HOST_ALIAS}([[:space:]]|$)" /etc/hosts; then
    if ! grep -qE "^${ROBOT_IP}[[:space:]]+${HOST_ALIAS}([[:space:]]|$)" /etc/hosts; then
        echo "[WARN] /etc/hosts has an existing ${HOST_ALIAS} entry that is not ${ROBOT_IP}."
        echo "       Updating it requires sudo."
        sudo sed -i.bak -E "s/^.*[[:space:]]${HOST_ALIAS}([[:space:]]|$)/${ROBOT_IP} ${HOST_ALIAS}/" /etc/hosts
    fi
else
    echo "[INFO] Adding ${HOST_ALIAS} to /etc/hosts..."
    echo "${ROBOT_IP} ${HOST_ALIAS}" | sudo tee -a /etc/hosts > /dev/null
fi

if grep -qE "^${ROBOT_IP}[[:space:]]+${HOST_ALIAS}([[:space:]]|$)" /etc/hosts; then
    echo "[OK] /etc/hosts maps ${HOST_ALIAS} to ${ROBOT_IP}"
else
    echo "[ERROR] /etc/hosts does not map ${HOST_ALIAS} to ${ROBOT_IP}"
    return 1 2>/dev/null || exit 1
fi

# 4. Activate ROS environment.
echo "[INFO] Activating ROS environment..."
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_VERSION ROS_DISTRO ROS_PYTHON_VERSION
conda activate ros_env

if [ $? -eq 0 ]; then
    echo "[OK] ROS environment activated"
else
    echo "[ERROR] Failed to activate ROS environment"
    return 1 2>/dev/null || exit 1
fi

# 5. Set ROS environment variables for wired mode.
export ROS_HOSTNAME="$LAPTOP_IP"
export ROS_MASTER_URI="$ROS_MASTER"
echo "[OK] ROS_HOSTNAME=$ROS_HOSTNAME"
echo "[OK] ROS_MASTER_URI=$ROS_MASTER_URI"

# 6. Test connection to robot.
echo "[INFO] Testing wired connection to robot..."
if ping -c 1 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "[OK] Robot is reachable at ${ROBOT_IP}"
else
    echo "[WARN] Cannot ping robot at ${ROBOT_IP}"
fi

# 7. Test ROS master TCP port before invoking rostopic.
echo "[INFO] Testing ROS master port..."
if nc -z -w 2 "$ROBOT_IP" 11311 > /dev/null 2>&1; then
    echo "[OK] ROS master port is open"
else
    echo "[ERROR] Cannot connect to ROS master at ${ROBOT_IP}:11311"
    echo "        The robot may not be in wired ROS mode or its ROS master may not be running."
    return 1 2>/dev/null || exit 1
fi

# 8. Test ROS graph.
echo "[INFO] Testing ROS connection..."
if timeout 5 rostopic list > /dev/null 2>&1; then
    echo "[OK] ROS connection successful!"
    echo ""
    echo "=== Ready to use! ==="
    echo "Example commands:"
    echo "  rostopic list"
    echo "  rostopic echo /odom"
    echo "  rostopic echo /livox/lidar"
else
    echo "[ERROR] Cannot query ROS topics from ${ROS_MASTER_URI}"
    return 1 2>/dev/null || exit 1
fi
