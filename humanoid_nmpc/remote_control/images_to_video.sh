#!/bin/bash
# Convert captured images to video
# Usage: ./images_to_video.sh [image_directory] [output_video] [fps]

IMAGE_DIR=${1:-"simulation_images_*"}
OUTPUT_VIDEO=${2:-"simulation_video_$(date +%Y%m%d_%H%M%S).webm"}
FPS=${3:-30}

echo "=========================================="
echo "Images to Video Converter"
echo "=========================================="

# Find the most recent image directory if pattern matches multiple
if [[ $IMAGE_DIR == *"*"* ]]; then
    IMAGE_DIR=$(ls -td $IMAGE_DIR 2>/dev/null | head -1)
fi

if [ -z "$IMAGE_DIR" ] || [ ! -d "$IMAGE_DIR" ]; then
    echo "Error: Image directory not found: $IMAGE_DIR"
    echo "Usage: ./images_to_video.sh [image_directory] [output_video] [fps]"
    exit 1
fi

# Count images (try both naming patterns)
IMAGE_COUNT=$(ls -1 "$IMAGE_DIR"/frame_*.png "$IMAGE_DIR"/rviz_frame_*.png 2>/dev/null | wc -l)

if [ "$IMAGE_COUNT" -eq 0 ]; then
    echo "Error: No images found in $IMAGE_DIR"
    echo "Looking for files matching: frame_*.png or rviz_frame_*.png"
    exit 1
fi

# Determine image pattern
if [ -f "$IMAGE_DIR/rviz_frame_000000.png" ]; then
    IMAGE_PATTERN="rviz_frame_%06d.png"
elif [ -f "$IMAGE_DIR/frame_000000.png" ]; then
    IMAGE_PATTERN="frame_%06d.png"
else
    # Try to find the pattern
    FIRST_IMAGE=$(ls -1 "$IMAGE_DIR"/*.png 2>/dev/null | head -1)
    if [ -z "$FIRST_IMAGE" ]; then
        echo "Error: No PNG images found"
        exit 1
    fi
    # Extract pattern from first image
    BASENAME=$(basename "$FIRST_IMAGE")
    if [[ $BASENAME == rviz_frame_*.png ]]; then
        IMAGE_PATTERN="rviz_frame_%06d.png"
    else
        IMAGE_PATTERN="frame_%06d.png"
    fi
fi

echo "Image directory: $IMAGE_DIR"
echo "Number of images: $IMAGE_COUNT"
echo "Output video: $OUTPUT_VIDEO"
echo "FPS: $FPS"
echo "=========================================="

# Check if ffmpeg is installed
if ! command -v ffmpeg &> /dev/null; then
    echo "Error: ffmpeg not found. Install with: sudo apt install ffmpeg"
    exit 1
fi

echo ""
echo "Converting images to video..."
echo ""

# Convert images to video
# Using detected pattern (6-digit zero-padded frame numbers)
echo "Using image pattern: $IMAGE_PATTERN"
ffmpeg -framerate $FPS \
       -i "$IMAGE_DIR/$IMAGE_PATTERN" \
       -vcodec libvpx-vp9 \
       -b:v 2M \
       -crf 30 \
       -pix_fmt yuv420p \
       "$OUTPUT_VIDEO"

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "Video created successfully!"
    echo "Output: $OUTPUT_VIDEO"
    echo "Duration: ~$((IMAGE_COUNT / FPS)) seconds"
    echo "=========================================="
else
    echo ""
    echo "Error: Failed to create video"
    exit 1
fi

