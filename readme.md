# Faze4 Robotic Arm — Hand Tracking Control

Hand-tracking based control system for the Faze4 robotic arm simulator in Unity.
The robot end-effector follows your hand position in real time using MediaPipe,
PyBullet inverse kinematics, and a live TCP camera stream to Unity.

This project extends the open-source Faze4 Unity simulator by
[kholodilinivan/Faze4-Robotic-arm-simulator](https://github.com/kholodilinivan/Faze4-Robotic-arm-simulator) with:
- Real-time hand tracking via webcam (MediaPipe)
- PyBullet IK solving for the Faze4 URDF
- TCP robot control stream to Unity (port 55001)
- TCP live camera feed stream to Unity (port 55002)
- Grab detection (open/closed hand)
- Z-axis depth control via wrist-to-MCP landmark distance
- **(v7) Exponential Moving Average (EMA) smoothing for jitter-free movement**

---

## 🤖 Smooth Control & Ergonomics (v7+)

**Problem:** Raw coordinates from MediaPipe hand tracking contain micro-fluctuations (jitter), causing the robot to shake even when the user holds their hand still. Additionally, the default vertical workspace was too high for comfortable desk use.

**Fix in v7:** - An **Exponential Moving Average (EMA) filter** is now applied to the target coordinates before they are sent to the IK solver. This averages out the micro-movements, resulting in smooth, stable, and fluid robot motion. The smoothing intensity can be tuned using the `alpha` variable (default `0.15`).
- `WORKSPACE_Z` is lowered to `-0.270` to shift the physical rest point of the robot downwards, making it more comfortable to control from a standard desk seated position without altering the actual movement scale.

## ⚠️ Resolution Calibration Fix (v6+)

**Problem:** The camera was calibrated with a high-resolution (2270×1514) chessboard image, but the live stream operates at a lower resolution (640×480). This mismatch caused the `MM_PER_PX` scaling factor to be approximately 3.5 times too small, which broke the Z-axis depth control.

**Fix in v6:** All calibration matrices and parameters have been mathematically scaled to match the 640×480 live resolution.
- `MM_PER_PX` is now `1.449 mm/px` (the metrically correct value for the stream resolution).
- `MOTION_SCALE` is now `0.000423` (recalculated to preserve the same robot motion range).
- `HAND_WRIST_TO_MCP_MM` (e.g., `80.0 mm`) now correctly corresponds to real-world hand size for accurate depth tracking.

---

## Repository Structure


```
Faze4-Robotic-arm-simulator/
├── UnityProject/                     ← Faze4 Unity simulator (original)
│   └── Assets/Scripts/
│       └── CameraStreamReceiver.cs   ← added: displays Python camera feed
├── faze.urdf                         ← robot model for PyBullet IK
├── calibration.ipynb                 ← camera calibration & mm/px scaling
├── v1_basic_xy_tracking.py
├── v2_grab_detection.py
├── v3_unity_camera_stream.py
├── v4_axis_correction_autoreconnect.py
├── v5_z_depth_control.py
├── v6_resolution_fixed.py
├── v7_smooth_control.py              ← latest version
├── CameraStreamReceiver.cs           ← Same script as in UnitProject folder for convenience
└── README.md
```

---

## Requirements

```bash
pip install opencv-python mediapipe pybullet numpy
```

Unity 2021+ with the Faze4 project open.

---

## Usage

1. Open the Unity project 
2. Assign `CameraStreamReceiver.cs` to a GameObject with a `RawImage` component
3. Press **Play** in the editor
4. Run the latest Python script:

```bash
python v7_smooth_control.py
```

The Python script and Unity will connect automatically — startup order does not matter.
Both TCP channels (robot control + camera stream) retry until the other side is ready.

---

## Hand Controls

| Gesture | Action |
| --- | --- |
| Open hand, move left/right | Robot Y-axis |
| Open hand, move up/down | Robot X-axis |
| Move hand toward/away from camera | Robot Z-axis (depth) |
| Close fist (< 2 fingers up) | Gripper close |
| Open hand | Gripper open |

---

## Key Parameters (`v7_smooth_control.py`)

| Parameter | Default | Description |
| --- | --- | --- |
| `alpha` (in IK loop) | `0.15` | Smoothing factor. Lower = smoother but slight delay, Higher = more responsive but more jitter. |
| `MOTION_SCALE` | `0.000423` | Amplifies XY hand motion to fit the robot's workspace for comfortable desk use. |
| `Z_SCALE` | `0.010` | Amplifies Z-axis (depth) hand motion for responsive control without large arm movements. |
| `HAND_WRIST_TO_MCP_MM` | `80.0` | Your physical wrist-to-knuckle distance (mm). Critical for Z-axis depth calibration. |
| `WORKSPACE_X_OFFSET` | `0.35` | Robot rest position X (meters) |
| `WORKSPACE_Y_OFFSET` | `-0.25` | Robot rest position Y (meters) |
| `WORKSPACE_Z` | `-0.270` | Robot neutral height Z (meters) - optimized for desk use. |
| `MIRROR_CAMERA` | `True` | Mirror feed so hand left = screen left |
| `JPEG_QUALITY` | `60` | Camera stream quality to Unity |

---

## Workspace Scaling and Control Logic

The system translates hand movements into robot commands through a multi-step scaling process designed for responsive and ergonomic desktop use.

1.  **Pixel to Millimeter Conversion:** The hand's wrist position in pixels $(x_{px}, y_{px})$ is first converted to physical millimeters in the camera's 2D plane using the corrected `MM_PER_PX` factor (`1.449 mm/px`). This provides a metric measurement of hand movement at the calibration distance.

2.  **XY-Axis Workspace Mapping:** The millimeter coordinates $(x_{mm}, y_{mm})$ are mapped to the robot's XY workspace. Instead of a direct 1:1 metric conversion (which would use a `MOTION_SCALE` of `0.001`), an empirically chosen `MOTION_SCALE` of `0.000423` is used.
    *   **Reasoning:** A true 1:1 scale would require impractically large hand movements to cover the robot's full range from a desk. The smaller `MOTION_SCALE` value amplifies hand motion, allowing the user to control the entire robot workspace with comfortable, small movements.

3.  **Z-Axis (Depth) Control:** Depth is estimated by measuring the apparent distance between the user's wrist and index finger knuckle (`d_img`) in the camera view. This is compared to a pre-measured reference distance (`d_ref`, e.g., 80mm). The difference is scaled by `Z_SCALE`.
    *   **Reasoning:** A `Z_SCALE` of `0.010` is used instead of a 1:1 scale (`0.001`). This amplifies the small, natural changes in hand-to-camera distance into a full, usable Z-axis range for the robot, avoiding the need for large forward/backward arm movements.

4.  **Inverse Kinematics (IK):** The final smoothed target position vector `[X, Y, Z]` is fed into the PyBullet IK solver, which calculates the required joint angles for the Faze4 arm. These angles are then streamed to Unity.

### A Note on Perspective and Accuracy

The XY-plane tracking is only metrically 1:1 accurate when your hand is at the specific distance from the camera used during calibration (~1 meter). This is a natural consequence of camera perspective:
*   **Further away (>1m):** A 10cm hand movement covers fewer pixels, resulting in a smaller robot movement.
*   **Closer (<1m):** The same hand movement covers more pixels and is amplified into a larger robot movement.

For true 1:1 spatial mapping across all depths, the `MOTION_SCALE` would need to be dynamically adjusted based on the estimated Z-distance. The current fixed-scaler approach prioritizes simplicity and intuitive control feel over absolute metric accuracy.

---

## Script Versions

| Script | Added feature |
| --- | --- |
| `v1_basic_xy_tracking.py` | XY hand tracking → robot, `cv2.imshow` window |
| `v2_grab_detection.py` | Grab detection via finger landmark comparison |
| `v3_unity_camera_stream.py` | Live camera feed streamed to Unity via TCP |
| `v4_axis_correction_autoreconnect.py` | Y-axis sign fix, mirroring, auto-reconnect on both TCP channels |
| `v5_z_depth_control.py` | Z-axis depth via wrist-MCP distance, personal hand calibration |
| `v6_resolution_fixed.py` | Math correctly scaled to 640x480 resolution. Fixed Z-axis tracking. |
| **`v7_smooth_control.py`** | **EMA smoothing (anti-jitter filter), ergonomic WORKSPACE_Z adjustment.** |

---

## Unity — CameraStreamReceiver.cs

Add this script to any GameObject in your scene. Assign a `RawImage` UI element
to the `displayImage` field in the Inspector. The script connects to the Python
camera server on port `55002` and auto-reconnects after disconnects.

---

## Credits

Unity Faze4 robot simulator based on:
[kholodilinivan/Faze4-Robotic-arm-simulator](https://github.com/kholodilinivan/Faze4-Robotic-arm-simulator)

Python hand tracking, IK solving, and Unity streaming written independently
on top of the simulator.