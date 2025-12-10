#!/bin/bash
# Simulation Video Recording Script
# Usage: ./record_simulation.sh [output_file] [fps] [window_name]

OUTPUT_FILE=${1:-"simulation_video_$(date +%Y%m%d_%H%M%S).webm"}
FPS=${2:-30}
WINDOW_NAME=${3:-"rviz2"}

echo "=========================================="
echo "Simulation Video Recorder"
echo "=========================================="
echo "Output: $OUTPUT_FILE"
echo "FPS: $FPS"
echo "Window: $WINDOW_NAME"
echo "=========================================="

# Check if ffmpeg is installed
if ! command -v ffmpeg &> /dev/null; then
    echo "Error: ffmpeg not found."
    echo "Install with: sudo apt install ffmpeg"
    exit 1
fi

# Check if xdotool is installed (for window detection)
if ! command -v xdotool &> /dev/null; then
    echo "Warning: xdotool not found. Will record entire screen."
    echo "Install with: sudo apt install xdotool"
    USE_WINDOW=false
else
    USE_WINDOW=true
fi

# Try to find window
if [ "$USE_WINDOW" = true ]; then
    WINDOW_ID=$(xdotool search --name "$WINDOW_NAME" 2>/dev/null | head -1)
    
    if [ -z "$WINDOW_ID" ]; then
        echo "Warning: Window '$WINDOW_NAME' not found."
        echo "Available windows:"
        xdotool search --name ".*" 2>/dev/null | head -5 | while read id; do
            xdotool getwindowname $id 2>/dev/null
        done
        echo ""
        echo "Recording entire screen instead..."
        USE_WINDOW=false
    fi
fi

if [ "$USE_WINDOW" = false ]; then
    echo ""
    echo "Recording entire screen..."
    echo "Press Ctrl+C to stop recording"
    echo ""
    ffmpeg -f x11grab \
           -framerate $FPS \
           -video_size 1920x1080 \
           -i :0.0 \
           -vcodec libvpx-vp9 \
           -b:v 2M \
           -crf 30 \
           "$OUTPUT_FILE"
else
    echo "Found window: $WINDOW_ID"
    # Get window geometry
    GEOM=$(xdotool getwindowgeometry $WINDOW_ID 2>/dev/null | grep Geometry | awk '{print $2}')
    if [ -z "$GEOM" ]; then
        echo "Could not get window geometry. Recording entire screen..."
        USE_WINDOW=false
    else
        WIDTH=$(echo $GEOM | cut -d'x' -f1)
        HEIGHT=$(echo $GEOM | cut -d'x' -f2)
        POS=$(xdotool getwindowgeometry $WINDOW_ID 2>/dev/null | grep Position | awk '{print $2}')
        X=$(echo $POS | cut -d',' -f1)
        Y=$(echo $POS | cut -d',' -f2)
        
        echo "Window size: ${WIDTH}x${HEIGHT}"
        echo "Window position: ${X},${Y}"
        echo ""
        echo "Press Ctrl+C to stop recording"
        echo ""
        
        ffmpeg -f x11grab \
               -framerate $FPS \
               -video_size ${WIDTH}x${HEIGHT} \
               -i :0.0+${X},${Y} \
               -vcodec libvpx-vp9 \
               -b:v 2M \
               -crf 30 \
               "$OUTPUT_FILE"
    fi
fi

if [ "$USE_WINDOW" = false ]; then
    # Fallback to full screen
    ffmpeg -f x11grab \
           -framerate $FPS \
           -video_size 1920x1080 \
           -i :0.0 \
           -vcodec libvpx-vp9 \
           -b:v 2M \
           -crf 30 \
           "$OUTPUT_FILE"
fi

echo ""
echo "=========================================="
echo "Recording complete!"
echo "Video saved to: $OUTPUT_FILE"
echo "=========================================="

