#!/bin/bash
# Simple image capture script
# Captures screenshots of a window and converts to video
# Usage: ./capture_images_simple.sh [window_name] [output_dir] [fps]

WINDOW_NAME=${1:-"rviz2"}
OUTPUT_DIR=${2:-"rviz_images_$(date +%Y%m%d_%H%M%S)"}
FPS=${3:-30}
INTERVAL=$(echo "scale=3; 1/$FPS" | bc)

echo "=========================================="
echo "Simple Image Capture"
echo "=========================================="
echo "Window: $WINDOW_NAME"
echo "Output: $OUTPUT_DIR"
echo "FPS: $FPS (interval: ${INTERVAL}s)"
echo "=========================================="

# Check dependencies
if ! command -v xdotool &> /dev/null; then
    echo "Error: xdotool not found. Install with: sudo apt install xdotool"
    exit 1
fi

# Try different screenshot tools
SCREENSHOT_TOOL=""
if command -v scrot &> /dev/null; then
    SCREENSHOT_TOOL="scrot"
elif command -v import &> /dev/null; then
    SCREENSHOT_TOOL="import"
elif command -v xwd &> /dev/null && command -v convert &> /dev/null; then
    SCREENSHOT_TOOL="xwd"
else
    echo "Error: No screenshot tool found!"
    echo "Install one of:"
    echo "  sudo apt install scrot"
    echo "  sudo apt install imagemagick"
    exit 1
fi

echo "Using: $SCREENSHOT_TOOL"

# Find window
WINDOW_ID=$(xdotool search --name "$WINDOW_NAME" 2>/dev/null | head -1)

if [ -z "$WINDOW_ID" ]; then
    echo "Error: Window '$WINDOW_NAME' not found!"
    exit 1
fi

echo "Found window: $WINDOW_ID"
echo "Creating output directory: $OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

echo ""
echo "Starting capture..."
echo "Press Ctrl+C to stop"
echo ""

FRAME=0

# Capture loop
while true; do
    FILENAME=$(printf "rviz_frame_%06d.png" $FRAME)
    OUTPUT_PATH="$OUTPUT_DIR/$FILENAME"
    
    case $SCREENSHOT_TOOL in
        scrot)
            # scrot can capture window directly
            scrot -u -b "$OUTPUT_PATH" 2>/dev/null
            ;;
        import)
            # ImageMagick import
            import -window "$WINDOW_ID" "$OUTPUT_PATH" 2>/dev/null
            ;;
        xwd)
            # xwd + convert
            xwd -id "$WINDOW_ID" | convert xwd:- "$OUTPUT_PATH" 2>/dev/null
            ;;
    esac
    
    if [ -f "$OUTPUT_PATH" ]; then
        FRAME=$((FRAME + 1))
        if [ $((FRAME % 30)) -eq 0 ]; then
            echo "Captured $FRAME frames..."
        fi
    fi
    
    sleep "$INTERVAL"
done

