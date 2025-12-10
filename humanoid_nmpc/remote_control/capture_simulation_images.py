#!/usr/bin/env python3
"""
Simulation Image Capture
Captures screenshots of the simulation window at regular intervals.
Images can later be converted to a video.
"""
import rclpy
from rclpy.node import Node
import subprocess
import time
import os
from datetime import datetime
import signal
import sys


class SimulationImageCapture(Node):
    def __init__(self, window_name="rviz2", output_dir=None, capture_rate=30.0):
        super().__init__("simulation_image_capture")
        
        self.window_name = window_name
        self.capture_interval = 1.0 / capture_rate  # seconds between captures
        self.capture_rate = capture_rate
        
        if output_dir is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_dir = f"rviz_images_{timestamp}"
        
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.frame_count = 0
        self.start_time = time.time()
        self.capturing = True
        self.window_id = None
        
        # Setup signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info("RViz Image Capture started")
        self.get_logger().info(f"Target window: {window_name}")
        self.get_logger().info(f"Output directory: {self.output_dir}")
        self.get_logger().info(f"Capture rate: {capture_rate} fps")
        self.get_logger().info("Press Ctrl+C to stop capturing")
        
        # Check dependencies
        if not self.check_dependencies():
            return
        
        # Find and verify window
        self.window_id = self.find_window()
        if not self.window_id:
            self.get_logger().error(f"RViz window '{window_name}' not found!")
            self.list_available_windows()
            return
        
        self.get_logger().info(f"Found RViz window: {self.window_id}")
        
        # Start capturing
        self.capture_loop()
    
    def check_dependencies(self):
        """Check if required tools are available"""
        try:
            subprocess.run(['xdotool', '--version'], 
                         capture_output=True, check=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            self.get_logger().error("xdotool not found. Install with: sudo apt install xdotool")
            return False
        
        try:
            subprocess.run(['import', '-version'], 
                         capture_output=True, check=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            # Try imagemagick import command
            try:
                subprocess.run(['which', 'import'], 
                             capture_output=True, check=True)
            except:
                self.get_logger().error("ImageMagick 'import' not found. Install with: sudo apt install imagemagick")
                return False
        
        return True
    
    def find_window(self):
        """Find the window ID"""
        try:
            result = subprocess.run(
                ['xdotool', 'search', '--name', self.window_name],
                capture_output=True,
                text=True,
                check=True
            )
            window_ids = result.stdout.strip().split('\n')
            if window_ids and window_ids[0]:
                return window_ids[0]
        except subprocess.CalledProcessError:
            pass
        return None
    
    def capture_window(self, window_id, output_path):
        """Capture a screenshot of the window"""
        try:
            # Use xdotool to get window geometry
            geom_result = subprocess.run(
                ['xdotool', 'getwindowgeometry', window_id],
                capture_output=True,
                text=True,
                check=True
            )
            
            # Parse geometry
            geom_line = [l for l in geom_result.stdout.split('\n') if 'Geometry' in l]
            pos_line = [l for l in geom_result.stdout.split('\n') if 'Position' in l]
            
            if not geom_line or not pos_line:
                return False
            
            geom = geom_line[0].split()[-1]  # e.g., "1920x1080"
            pos = pos_line[0].split()[-1]    # e.g., "100,200"
            
            width, height = geom.split('x')
            x, y = pos.split(',')
            
            # Use ImageMagick import to capture the window
            # Format: import -window <window_id> output.png
            subprocess.run(
                ['import', '-window', window_id, output_path],
                capture_output=True,
                check=True,
                timeout=5
            )
            return True
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, ValueError) as e:
            self.get_logger().warn(f"Failed to capture frame {self.frame_count}: {e}")
            return False
    
    def list_available_windows(self):
        """List available windows for debugging"""
        self.get_logger().info("Available windows:")
        try:
            result = subprocess.run(
                ['xdotool', 'search', '--name', '.*'],
                capture_output=True,
                text=True
            )
            window_ids = result.stdout.strip().split('\n')[:15]
            for wid in window_ids:
                if wid:
                    try:
                        name_result = subprocess.run(
                            ['xdotool', 'getwindowname', wid],
                            capture_output=True,
                            text=True
                        )
                        name = name_result.stdout.strip()
                        if name:
                            self.get_logger().info(f"  - {name} (ID: {wid})")
                    except:
                        pass
        except:
            pass
    
    def capture_loop(self):
        """Main capture loop"""
        if not self.window_id:
            return
        
        self.get_logger().info("Starting RViz window capture...")
        
        last_capture_time = time.time()
        
        while self.capturing:
            current_time = time.time()
            
            if current_time - last_capture_time >= self.capture_interval:
                # Generate filename with zero-padded frame number
                filename = f"rviz_frame_{self.frame_count:06d}.png"
                output_path = os.path.join(self.output_dir, filename)
                
                if self.capture_window(self.window_id, output_path):
                    self.frame_count += 1
                    if self.frame_count % 30 == 0:  # Log every 30 frames
                        elapsed = current_time - self.start_time
                        self.get_logger().info(
                            f"Captured {self.frame_count} RViz images "
                            f"({self.frame_count/elapsed:.1f} fps avg)"
                        )
                
                last_capture_time = current_time
            
            # Small sleep to avoid busy waiting
            time.sleep(0.01)
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C"""
        self.get_logger().info(f"\nStopping capture... Captured {self.frame_count} RViz images")
        self.capturing = False
        self.get_logger().info(f"Images saved in: {self.output_dir}")
        self.get_logger().info(f"To create video, run:")
        self.get_logger().info(f"  ./images_to_video.sh {self.output_dir} output.webm {self.capture_rate}")
        sys.exit(0)


def main(args=None):
    import argparse
    
    parser = argparse.ArgumentParser(description='Capture simulation window screenshots')
    parser.add_argument('--window', '-w', type=str, default='rviz2',
                       help='Window name to capture (default: rviz2)')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='Output directory (default: simulation_images_TIMESTAMP)')
    parser.add_argument('--fps', type=float, default=30.0,
                       help='Capture rate in fps (default: 30)')
    
    rclpy.init(args=args)
    
    if args is None:
        args = sys.argv[1:]
    
    parsed_args = parser.parse_args(args)
    
    node = SimulationImageCapture(
        window_name=parsed_args.window,
        output_dir=parsed_args.output,
        capture_rate=parsed_args.fps
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

