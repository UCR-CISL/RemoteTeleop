#!/bin/bash
# START.sh - Configure and connect to the Lubao delivery robot
# Usage: source START.sh

# Check if script is being sourced (not executed directly)
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "[WARN] This script should be sourced, not executed directly!"
    echo "       Run: source START.sh"
    echo ""
    echo "Continuing anyway, but environment won't persist to your shell..."
    echo ""
fi

echo "=== Lubao Robot Connection Setup ==="

# 0. Initialize conda/mamba - always source the init scripts
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

# Initialize mamba if available
if [ -f "$HOME/anaconda3/etc/profile.d/mamba.sh" ]; then
    source "$HOME/anaconda3/etc/profile.d/mamba.sh"
elif [ -f "$HOME/miniforge3/etc/profile.d/mamba.sh" ]; then
    source "$HOME/miniforge3/etc/profile.d/mamba.sh"
fi

# 1. Check if USB WiFi is connected to robot hotspot
ROBOT_IP=$(ip addr show wlx145d34bd15fd 2>/dev/null | grep "inet 10.42.0" | awk '{print $2}' | cut -d'/' -f1)

if [ -z "$ROBOT_IP" ]; then
    echo "[ERROR] USB WiFi adapter not connected to robot hotspot!"
    echo "Please connect 'wlx145d34bd15fd' to WiFi: R100A2501AZF2V00"
    echo "Password: R100A2501AZF2V00"
    return 1 2>/dev/null || exit 1
fi

echo "[OK] USB WiFi connected with IP: $ROBOT_IP"

# 2. Fix routing - remove robot's default gateway if it exists
if ip route | grep -q "default via 10.42.0.1"; then
    echo "[INFO] Removing robot's default gateway (requires sudo)..."
    sudo ip route del default via 10.42.0.1 dev wlx145d34bd15fd
    if [ $? -eq 0 ]; then
        echo "[OK] Routing fixed"
    else
        echo "[ERROR] Failed to fix routing"
        return 1 2>/dev/null || exit 1
    fi
else
    echo "[OK] Routing already correct"
fi

# 3. Activate ROS environment
echo "[INFO] Activating ROS environment..."
conda activate ros_env

if [ $? -eq 0 ]; then
    echo "[OK] ROS environment activated"
else
    echo "[ERROR] Failed to activate ROS environment"
    return 1 2>/dev/null || exit 1
fi

# 4. Set ROS environment variables
export ROS_HOSTNAME="$ROBOT_IP"
export ROS_MASTER_URI="http://10.42.0.1:11311"
echo "[OK] ROS_HOSTNAME=$ROS_HOSTNAME"
echo "[OK] ROS_MASTER_URI=$ROS_MASTER_URI"

# 5. Test connection to robot
echo "[INFO] Testing connection to robot..."
if ping -c 1 -W 2 10.42.0.1 > /dev/null 2>&1; then
    echo "[OK] Robot is reachable"
else
    echo "[WARN] Cannot ping robot - it may still work"
fi

# 6. Test ROS connection
echo "[INFO] Testing ROS connection..."
if timeout 5 rostopic list > /dev/null 2>&1; then
    echo "[OK] ROS connection successful!"
    echo ""
    echo "=== Ready to use! ==="
    echo "Example commands:"
    echo "  rostopic list              # List all topics"
    echo "  rostopic echo /odom        # View odometry"
    echo "  rostopic echo /livox/lidar # View 3D LiDAR"
    echo ""
else
    echo "[ERROR] Cannot connect to ROS master"
    echo "Make sure the robot is powered on"
    return 1 2>/dev/null || exit 1
fi
