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

1. Open the Unity project assign `CameraStreamReceiver.cs` to a GameObject with a `RawImage` component
2. Press **Play** in the Editor
3. Run the latest Python script:

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
| `MOTION_SCALE` | `0.000423` | XY amplification — matched to resolution calibration |
| `Z_SCALE` | `0.010` | Z-axis sensitivity — increase for more depth response |
| `HAND_WRIST_TO_MCP_MM` | `80.0` | Measured wrist crease to index finger knuckle in mm |
| `WORKSPACE_X_OFFSET` | `0.35` | Robot rest position X (meters) |
| `WORKSPACE_Y_OFFSET` | `-0.25` | Robot rest position Y (meters) |
| `WORKSPACE_Z` | `-0.270` | Robot neutral height Z (meters) - optimized for desk use |
| `MIRROR_CAMERA` | `True` | Mirror feed so hand left = screen left |
| `JPEG_QUALITY` | `60` | Camera stream quality to Unity |

---

## Workspace Scaling

`MM_PER_PX = 0.4085` converts wrist pixel position to physical mm in the camera
plane. `MOTION_SCALE` then maps those mm values to robot workspace meters.

### A Note on XY Tracking Accuracy and Depth

The `MM_PER_PX` conversion factor is calculated from a calibration image taken at a fixed distance (e.g., ~1 meter). This means the XY-plane tracking is only metrically 1:1 accurate when your hand is at that specific distance from the camera.

**This is a perspective effect, not a bug.** Because of camera projection, an object's size in pixels shrinks as its distance from the camera increases.

* **At the calibration distance (~1m):** A 10cm hand movement correctly translates to a 10cm equivalent robot movement.
* **Further away (>1m):** The same 10cm hand movement covers fewer pixels, resulting in a smaller-than-10cm robot movement.
* **Closer (<1m):** The hand movement will be amplified, resulting in a larger robot movement.

The current system uses this fixed scaler for simplicity. For true 1:1 tracking across different depths, the XY scaling would need to be dynamically adjusted based on the calculated Z-distance. This correction is possible and could be implemented in a future version for applications requiring higher spatial accuracy.

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