#!/usr/bin/env python3
"""
Simulation Video Recorder
Records robot simulation videos by capturing RViz visualization or screen.
Supports multiple recording methods.
"""
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from datetime import datetime
import argparse
import sys
import subprocess
import os
import time


class SimulationVideoRecorder(Node):
    def __init__(self, output_file=None, fps=30, method='rosbag'):
        super().__init__("simulation_video_recorder")
        
        self.fps = fps
        self.method = method
        
        if output_file is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_file = f"simulation_video_{timestamp}.webm"
        
        self.output_file = output_file
        self.recording = False
        self.video_writer = None
        
        self.get_logger().info(f"Simulation Video Recorder initialized")
        self.get_logger().info(f"Output file: {self.output_file}")
        self.get_logger().info(f"Method: {self.method}")
        self.get_logger().info(f"FPS: {self.fps}")
    
    def start_recording_rosbag(self):
        """Start recording using rosbag2"""
        self.get_logger().info("Starting rosbag recording...")
        self.get_logger().info("This will record all topics. You can replay later with video recording.")
        self.get_logger().info(f"Recording to: {self.output_file.replace('.mp4', '.db3')}")
        
        # Note: rosbag2 recording should be done separately
        # This is just a helper message
        cmd = f"ros2 bag record -o {self.output_file.replace('.mp4', '')} -a"
        self.get_logger().info(f"Run this command in another terminal: {cmd}")
        return cmd
    
    def start_recording_ffmpeg(self, window_name="rviz2"):
        """Start recording using ffmpeg (screen capture)"""
        self.get_logger().info(f"Starting ffmpeg screen recording of window: {window_name}")
        
        # Check if ffmpeg is available
        try:
            subprocess.run(['ffmpeg', '-version'], capture_output=True, check=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            self.get_logger().error("ffmpeg not found! Please install: sudo apt install ffmpeg")
            return None
        
        # Get window ID
        try:
            result = subprocess.run(
                ['xdotool', 'search', '--name', window_name],
                capture_output=True,
                text=True,
                check=True
            )
            window_id = result.stdout.strip().split('\n')[0]
        except (subprocess.CalledProcessError, FileNotFoundError, IndexError):
            self.get_logger().warn("xdotool not found or window not found. Trying alternative method...")
            # Alternative: record entire screen or use xwininfo
            window_id = None
        
        if window_id:
            # Record specific window
            cmd = [
                'ffmpeg', '-f', 'x11grab',
                '-framerate', str(self.fps),
                '-s', '1920x1080',  # Adjust as needed
                '-i', f':0.0+0,0',  # Screen coordinates, adjust as needed
                '-vcodec', 'libx264',
                '-preset', 'medium',
                '-crf', '23',
                '-pix_fmt', 'yuv420p',
                self.output_file
            ]
        else:
            # Record entire screen (fallback)
            cmd = [
                'ffmpeg', '-f', 'x11grab',
                '-framerate', str(self.fps),
                '-s', '1920x1080',
                '-i', ':0.0',
                '-vcodec', 'libx264',
                '-preset', 'medium',
                '-crf', '23',
                '-pix_fmt', 'yuv420p',
                self.output_file
            ]
        
        self.get_logger().info(f"Run this command: {' '.join(cmd)}")
        self.get_logger().info("Or use the provided helper script")
        return cmd
    
    def start_recording_image_topic(self, topic_name="/rviz/image"):
        """Start recording from image topic (if RViz publishes images)"""
        self.get_logger().info(f"Recording from image topic: {topic_name}")
        self.get_logger().warn("This method requires RViz to publish images. May not be available by default.")
        
        # This would require subscribing to image topic
        # For now, just provide instructions
        self.get_logger().info("To enable this, configure RViz to publish images or use screen capture method")
        return None


def create_recording_script():
    """Create a helper bash script for recording"""
    script_content = """#!/bin/bash
# Simulation Video Recording Script
# Usage: ./record_simulation.sh [output_file] [fps] [window_name]

OUTPUT_FILE=${1:-"simulation_video_$(date +%Y%m%d_%H%M%S).mp4"}
FPS=${2:-30}
WINDOW_NAME=${3:-"rviz2"}

echo "Recording simulation video..."
echo "Output: $OUTPUT_FILE"
echo "FPS: $FPS"
echo "Window: $WINDOW_NAME"

# Check if ffmpeg is installed
if ! command -v ffmpeg &> /dev/null; then
    echo "Error: ffmpeg not found. Install with: sudo apt install ffmpeg"
    exit 1
fi

# Try to find window
WINDOW_ID=$(xdotool search --name "$WINDOW_NAME" 2>/dev/null | head -1)

if [ -z "$WINDOW_ID" ]; then
    echo "Warning: Window '$WINDOW_NAME' not found. Recording entire screen."
    echo "Press Ctrl+C to stop recording..."
    ffmpeg -f x11grab -framerate $FPS -s 1920x1080 -i :0.0 \\
           -vcodec libx264 -preset medium -crf 23 -pix_fmt yuv420p \\
           "$OUTPUT_FILE"
else
    echo "Found window: $WINDOW_ID"
    echo "Press Ctrl+C to stop recording..."
    # Get window geometry
    GEOM=$(xdotool getwindowgeometry $WINDOW_ID | grep Geometry | awk '{print $2}')
    WIDTH=$(echo $GEOM | cut -d'x' -f1)
    HEIGHT=$(echo $GEOM | cut -d'x' -f2)
    POS=$(xdotool getwindowgeometry $WINDOW_ID | grep Position | awk '{print $2}')
    X=$(echo $POS | cut -d',' -f1)
    Y=$(echo $POS | cut -d',' -f2)
    
    ffmpeg -f x11grab -framerate $FPS -s ${WIDTH}x${HEIGHT} -i :0.0+${X},${Y} \\
           -vcodec libx264 -preset medium -crf 23 -pix_fmt yuv420p \\
           "$OUTPUT_FILE"
fi

echo "Recording complete: $OUTPUT_FILE"
"""
    return script_content


def main(args=None):
    parser = argparse.ArgumentParser(description='Record robot simulation videos')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='Output video file (default: simulation_video_TIMESTAMP.webm)')
    parser.add_argument('--fps', type=int, default=30,
                       help='Frames per second (default: 30)')
    parser.add_argument('--method', type=str, choices=['rosbag', 'ffmpeg', 'image_topic'],
                       default='ffmpeg', help='Recording method (default: ffmpeg)')
    parser.add_argument('--window', type=str, default='rviz2',
                       help='Window name to record (default: rviz2)')
    parser.add_argument('--create-script', action='store_true',
                       help='Create a helper bash script for recording')
    
    rclpy.init(args=args)
    
    # Parse arguments
    if args is None:
        args = sys.argv[1:]
    parsed_args = parser.parse_args(args)
    
    if parsed_args.create_script:
        script_content = create_recording_script()
        script_path = "record_simulation.sh"
        with open(script_path, 'w') as f:
            f.write(script_content)
        os.chmod(script_path, 0o755)
        print(f"Created recording script: {script_path}")
        print("Usage: ./record_simulation.sh [output_file] [fps] [window_name]")
        return
    
    node = SimulationVideoRecorder(
        output_file=parsed_args.output,
        fps=parsed_args.fps,
        method=parsed_args.method
    )
    
    if parsed_args.method == 'rosbag':
        cmd = node.start_recording_rosbag()
        print(f"\nTo record, run in another terminal:")
        print(cmd)
    elif parsed_args.method == 'ffmpeg':
        cmd = node.start_recording_ffmpeg(parsed_args.window)
        if cmd:
            print(f"\nTo record, run:")
            print(' '.join(cmd))
            print(f"\nOr use the helper script: ./record_simulation.sh {parsed_args.output} {parsed_args.fps} {parsed_args.window}")
    elif parsed_args.method == 'image_topic':
        node.start_recording_image_topic()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

