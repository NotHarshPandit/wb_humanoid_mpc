#!/bin/bash
# Record a specific window by name
# Usage: ./record_window.sh [window_name] [output_file] [fps]

WINDOW_NAME=${1:-"rviz2"}
OUTPUT_FILE=${2:-"window_recording_$(date +%Y%m%d_%H%M%S).webm"}
FPS=${3:-30}

echo "=========================================="
echo "Window Recorder"
echo "=========================================="
echo "Window: $WINDOW_NAME"
echo "Output: $OUTPUT_FILE"
echo "FPS: $FPS"
echo "=========================================="

# Check dependencies
if ! command -v ffmpeg &> /dev/null; then
    echo "Error: ffmpeg not found. Install with: sudo apt install ffmpeg"
    exit 1
fi

if ! command -v xdotool &> /dev/null; then
    echo "Error: xdotool not found. Install with: sudo apt install xdotool"
    exit 1
fi

# Find window
WINDOW_ID=$(xdotool search --name "$WINDOW_NAME" 2>/dev/null | head -1)

if [ -z "$WINDOW_ID" ]; then
    echo "Error: Window '$WINDOW_NAME' not found!"
    echo ""
    echo "Available windows:"
    xdotool search --name ".*" 2>/dev/null | head -20 | while read id; do
        name=$(xdotool getwindowname $id 2>/dev/null)
        if [ ! -z "$name" ]; then
            echo "  - $name (ID: $id)"
        fi
    done
    exit 1
fi

echo "Found window: $WINDOW_ID"

# Activate window to ensure it's visible
xdotool windowactivate $WINDOW_ID 2>/dev/null
sleep 0.5

# Get window geometry
GEOM_INFO=$(xdotool getwindowgeometry $WINDOW_ID 2>/dev/null)
GEOM=$(echo "$GEOM_INFO" | grep Geometry | awk '{print $2}')
POS=$(echo "$GEOM_INFO" | grep Position | awk '{print $2}')

if [ -z "$GEOM" ] || [ -z "$POS" ]; then
    echo "Error: Could not get window geometry"
    exit 1
fi

WIDTH=$(echo $GEOM | cut -d'x' -f1)
HEIGHT=$(echo $GEOM | cut -d'x' -f2)
X=$(echo $POS | cut -d',' -f1)
Y=$(echo $POS | cut -d',' -f2)

echo "Window geometry: ${WIDTH}x${HEIGHT} at position (${X},${Y})"

# Validate size
if [ "$WIDTH" -lt 100 ] || [ "$HEIGHT" -lt 100 ]; then
    echo "Warning: Window size is very small (${WIDTH}x${HEIGHT})"
    echo "The window might be minimized. Please maximize it and try again."
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

DISPLAY=${DISPLAY:-:0}

echo ""
echo "Starting recording..."
echo "Press Ctrl+C to stop"
echo ""

# Record the window
ffmpeg -f x11grab \
       -framerate $FPS \
       -video_size ${WIDTH}x${HEIGHT} \
       -i ${DISPLAY}+${X},${Y} \
       -vcodec libvpx-vp9 \
       -b:v 2M \
       -crf 30 \
       "$OUTPUT_FILE"

echo ""
echo "=========================================="
echo "Recording complete!"
echo "Video saved to: $OUTPUT_FILE"
echo "=========================================="


