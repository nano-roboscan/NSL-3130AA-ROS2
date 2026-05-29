#!/usr/bin/env bash
# Extrinsic calibration for NSL-3130AA (LiDAR <-> Camera).
#
# Usage: ./extrinsic_calib.sh [camera_id] [image_topic] [lidar_topic]
#   camera_id   : camera serial or ID; auto-detected from calib_output if omitted
#   image_topic : ROS topic for RGB image   (default /camera/rgb/image_raw)
#   lidar_topic : ROS topic for point cloud (default /camera/point_cloud)
#
# Requires:
#   1. camera.launch.py running (provides image + point_cloud topics)
#   2. intrinsic_calib.sh completed first
#
# Interactive keys (once running):
#   [s] store frame  [c] calibrate  [r] reset  [q] quit
#
# Output: <repo>/calib_output/<camera_id>_extrinsic.yml

CAMERA_ID="${1:-}"
IMAGE_TOPIC="${2:-/camera/rgb/image_raw}"
LIDAR_TOPIC="${3:-/camera/point_cloud}"

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALIB_DIR="$REPO_DIR/calib_output"
DETECT_SCRIPT="$REPO_DIR/NSL3130_driver/src/roboscan_nsl3130/scripts/detect_camera_id.py"

source /opt/ros/humble/setup.bash
WS_SETUP="$REPO_DIR/../../install/setup.bash"
[ -f "$WS_SETUP" ] && source "$WS_SETUP"

# Auto-detect: USB serial first, then fall back to intrinsic.yml filename
if [ -z "$CAMERA_ID" ]; then
    CAMERA_ID=$(python3 "$DETECT_SCRIPT" 2>/dev/null)
    if [ -n "$CAMERA_ID" ]; then
        echo "[extrinsic] Auto-detected camera_id: $CAMERA_ID"
    else
        YML=$(ls "$CALIB_DIR"/*_intrinsic.yml 2>/dev/null | head -1)
        if [ -z "$YML" ]; then
            echo "[ERROR] Camera not detected via USB and no *_intrinsic.yml found in $CALIB_DIR"
            echo "        Connect the camera, or run ./intrinsic_calib.sh first."
            exit 1
        fi
        CAMERA_ID=$(basename "$YML" _intrinsic.yml)
        echo "[extrinsic] Using camera_id from intrinsic.yml: $CAMERA_ID"
    fi
fi

echo ""
echo "  Camera: $CAMERA_ID    Image: $IMAGE_TOPIC    LiDAR: $LIDAR_TOPIC"
echo "  Output: $CALIB_DIR"
echo ""

ros2 run roboscan_nsl3130 extrinsic_calibration_node.py \
    --camera-id   "$CAMERA_ID" \
    --image-topic "$IMAGE_TOPIC" \
    --lidar-topic "$LIDAR_TOPIC" \
    --output-dir  "$CALIB_DIR"
