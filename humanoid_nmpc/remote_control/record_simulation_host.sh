#!/bin/bash
# Simulation Video Recording Script (Run from HOST machine)
# This script records the RViz window from your host machine
# Usage: ./record_simulation_host.sh [output_file] [fps]

OUTPUT_FILE=${1:-"simulation_video_$(date +%Y%m%d_%H%M%S).webm"}
FPS=${2:-30}

echo "=========================================="
echo "Simulation Video Recorder (Host)"
echo "=========================================="
echo "Output: $OUTPUT_FILE"
echo "FPS: $FPS"
echo "=========================================="

# Check if ffmpeg is installed
if ! command -v ffmpeg &> /dev/null; then
    echo "Error: ffmpeg not found."
    echo "Install with: sudo apt install ffmpeg"
    exit 1
fi

# Check if xdotool is installed
if ! command -v xdotool &> /dev/null; then
    echo "Warning: xdotool not found. Will record entire screen."
    echo "Install with: sudo apt install xdotool"
    USE_WINDOW=false
else
    USE_WINDOW=true
fi

# Try to find RViz window
if [ "$USE_WINDOW" = true ]; then
    WINDOW_ID=$(xdotool search --name "rviz2" 2>/dev/null | head -1)
    
    if [ -z "$WINDOW_ID" ]; then
        echo "RViz2 window not found. Trying other window names..."
        WINDOW_ID=$(xdotool search --name "RViz" 2>/dev/null | head -1)
    fi
    
    if [ -z "$WINDOW_ID" ]; then
        echo "Warning: RViz window not found."
        echo "Available windows:"
        xdotool search --name ".*" 2>/dev/null | head -10 | while read id; do
            name=$(xdotool getwindowname $id 2>/dev/null)
            if [ ! -z "$name" ]; then
                echo "  - $name"
            fi
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
    DISPLAY=${DISPLAY:-:0}
    SCREEN_SIZE=$(xrandr | grep '*' | head -1 | awk '{print $1}')
    if [ -z "$SCREEN_SIZE" ]; then
        SCREEN_SIZE="1920x1080"
    fi
    ffmpeg -f x11grab \
           -framerate $FPS \
           -video_size $SCREEN_SIZE \
           -i ${DISPLAY} \
           -vcodec libvpx-vp9 \
           -b:v 2M \
           -crf 30 \
           "$OUTPUT_FILE"
else
    echo "Found RViz window: $WINDOW_ID"
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
        
        # Check if window size is valid (not too small)
        if [ "$WIDTH" -lt 100 ] || [ "$HEIGHT" -lt 100 ]; then
            echo "Window size too small (${WIDTH}x${HEIGHT}). Window might be minimized."
            echo "Please maximize the RViz window and try again."
            echo ""
            echo "You can also manually specify window to record:"
            echo "  xdotool search --name 'rviz2' | head -1 | xargs xdotool windowactivate"
            echo "Recording entire screen instead..."
            USE_WINDOW=false
        else
            echo "Window size: ${WIDTH}x${HEIGHT}"
            echo "Window position: ${X},${Y}"
            echo ""
            echo "Recording ONLY the RViz window..."
            echo "Press Ctrl+C to stop recording"
            echo ""
            
            # Fix display format: use :0.0 and offset
            DISPLAY=${DISPLAY:-:0}
            # Use x11grab with offset to capture only the window
            ffmpeg -f x11grab \
                   -framerate $FPS \
                   -video_size ${WIDTH}x${HEIGHT} \
                   -i ${DISPLAY}+${X},${Y} \
                   -vcodec libvpx-vp9 \
                   -b:v 2M \
                   -crf 30 \
                   "$OUTPUT_FILE"
        fi
    fi
fi

if [ "$USE_WINDOW" = false ]; then
    # Fallback to full screen
    DISPLAY=${DISPLAY:-:0}
    echo ""
    echo "Recording entire screen..."
    echo "Press Ctrl+C to stop recording"
    echo ""
    
    # Get screen resolution
    SCREEN_SIZE=$(xrandr | grep '*' | head -1 | awk '{print $1}')
    if [ -z "$SCREEN_SIZE" ]; then
        SCREEN_SIZE="1920x1080"  # Default
    fi
    
    echo "Screen size: $SCREEN_SIZE"
    echo ""
    
    ffmpeg -f x11grab \
           -framerate $FPS \
           -video_size $SCREEN_SIZE \
           -i ${DISPLAY} \
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

