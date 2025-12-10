# Recording Robot Simulation Videos

Since `ffmpeg` and `xdotool` are not available in the Docker container, here are alternative methods to record simulation videos:

## Method 1: Record from Host Machine (Recommended)

**Step 1: Install dependencies on your HOST machine (not in Docker)**
```bash
# On your host (outside Docker)
sudo apt install ffmpeg xdotool
```

**Step 2: Run the recording script from host**
```bash
# On your host machine
cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc/humanoid_nmpc/remote_control
./record_simulation_host.sh [output_file] [fps]
```

**Example:**
```bash
./record_simulation_host.sh my_simulation.mp4 30
```

The script will automatically find the RViz window and record it.

---

## Method 2: Use rosbag2 (Record topics, replay later)

**Step 1: Record all topics**
```bash
# In Docker container
cd /wb_humanoid_mpc_ws
ros2 bag record -o simulation_recording -a
```

**Step 2: Stop recording** (Ctrl+C)

**Step 3: Replay and record from host**
```bash
# Terminal 1: Replay the bag
cd /wb_humanoid_mpc_ws
ros2 bag play simulation_recording

# Terminal 2 (on HOST): Record the screen
./record_simulation_host.sh replay_video.mp4
```

---

## Method 3: Use OBS Studio (GUI tool)

1. Install OBS Studio on your host machine:
   ```bash
   sudo apt install obs-studio
   ```

2. Open OBS Studio
3. Add a "Window Capture" source
4. Select the RViz window
5. Click "Start Recording"

---

## Method 4: Simple Screen Recording (Host)

If you just want to record the entire screen:

```bash
# On your host machine
ffmpeg -f x11grab -framerate 30 -video_size 1920x1080 -i :0.0 \
       -vcodec libx264 -preset medium -crf 23 -pix_fmt yuv420p \
       simulation_video.mp4
```

Press `Ctrl+C` to stop.

---

## Method 5: Record Specific Window (Host)

1. Find window ID:
   ```bash
   xdotool search --name "rviz2"
   ```

2. Get window position and size:
   ```bash
   xdotool getwindowgeometry <WINDOW_ID>
   ```

3. Record that window:
   ```bash
   ffmpeg -f x11grab -framerate 30 \
          -video_size WIDTHxHEIGHT \
          -i :0.0+X,Y \
          -vcodec libx264 -preset medium -crf 23 \
          simulation_video.mp4
   ```

Replace `WIDTH`, `HEIGHT`, `X`, `Y` with actual values.

---

## Quick Start (Recommended)

**On your HOST machine:**

1. Install dependencies:
   ```bash
   sudo apt install ffmpeg xdotool
   ```

2. Start your simulation (in Docker)

3. Record:
   ```bash
   cd ~/humanoid_mpc_ws/src/wb_humanoid_mpc/humanoid_nmpc/remote_control
   ./record_simulation_host.sh
   ```

4. Stop recording: Press `Ctrl+C`

The video will be saved in the current directory.

