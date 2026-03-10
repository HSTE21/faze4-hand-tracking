import cv2
import mediapipe as mp
import numpy as np
import socket
import struct                          
from queue import Queue, Empty
import time
import threading
import pybullet as p
import pybullet_data
import os


# Increase for more robot range with the same hand movement
# 0.0008 = small range, 0.0014 = larger range
MOTION_SCALE = 0.0010


# Rest position of the robot (where the arm is when the hand is at x=0, y=0)
# Adjust if the robot moves outside its workspace
WORKSPACE_X_OFFSET =  0.35   # meters — forward/backward
WORKSPACE_Y_OFFSET = -0.25   # meters — left/right
WORKSPACE_Z        =  0.197  # meters — fixed grip height


# Mirrors the camera so that hand to the left = screen to the left
MIRROR_CAMERA = True


# JPEG quality for Unity stream (lower = faster, higher = sharper)
JPEG_QUALITY = 60



# === CALIBRATION ===
MM_PER_PX = 0.4085
# Camera calibration parameters for undistortion
mtx = np.array([[2.00313622e+03, 0.00000000e+00, 1.13719259e+03],
                [0.00000000e+00, 2.00602079e+03, 7.34877818e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.array([[-2.57939135e-02,  3.50765930e-01, -1.66865917e-04, -7.56033165e-04,
                  -8.65799370e-01]])
newcameramtx = np.array([[1.93156188e+03, 0.00000000e+00, 1.13553543e+03],
                         [0.00000000e+00, 1.94087193e+03, 7.31467517e+02],
                         [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
roi = (39, 22, 2190, 1460)
# Precompute undistort maps for efficiency
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
xy_queue = Queue(maxsize=1)
frame_queue = Queue(maxsize=1)

# Global stop signal for a clean exit via Ctrl+C
stop_event = threading.Event()



# === UNITY TCP ===
def tcp_init(host='127.0.0.1', port=55001, retry_interval=2.0):
    while not stop_event.is_set():
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((host, port))
            print(f"✅ Connected Unity {host}:{port}")
            return sock
        except ConnectionRefusedError:
            print(f"⏳ Waiting for Unity ({host}:{port})...")
            sock.close()
            time.sleep(retry_interval)




def func_data(client, q_deg):  # Direct degrees!
    q_deg = np.clip(q_deg, -170, 170)  # Unity safe limits
    msg = f"ModRobot:{','.join(f'{qi:.3f}' for qi in q_deg)}"
    data = bytes([len(msg)]) + msg.encode('utf-8')
    client.send(data)
    # Print line temporarily disabled so your console doesn't get flooded, feel free to re-enable!
    # print(f"✅ Sent: J1={q_deg[0]:.1f}° J5={q_deg[4]:.1f}°")



def func_grab(client, state):
    msg = f"ModGrab:{int(state)}"
    data = bytes([len(msg)]) + msg.encode('utf-8')
    client.send(data)



# Camera stream server for Unity
# Restarts automatically after disconnect so Unity can always reconnect
def camera_stream_server(port=55002):
    while not stop_event.is_set():
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', port))
        server.listen(1)
        print(f"📷 Waiting for Unity camera connection on port {port}...")
        try:
            server.settimeout(2.0)
            conn, addr = server.accept()
            print(f"📷 Unity camera connected: {addr}")
            conn.settimeout(1.0)
            while not stop_event.is_set():
                try:
                    # Blocking get prevents busy-loop and saves CPU
                    frame = frame_queue.get(timeout=0.05)
                except Empty:
                    continue
                ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ret:
                    data = buf.tobytes()
                    conn.sendall(struct.pack('>I', len(data)) + data)
        except socket.timeout:
            pass
        except Exception as e:
            print(f"📷 Camera connection lost: {e}")
        finally:
            try: conn.close()
            except: pass
            server.close()



# === PYBULLET ROBOT (Faze4) ===
def init_pybullet_robot():
    p.connect(p.DIRECT)  # No GUI - fast computation
    p.setGravity(0, 0, -9.81)
    
    # Construct an absolute path to the URDF file to prevent "file not found" errors.
    # This works even if the script is run from a different working directory.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "faze4.urdf")

    # Load the new Faze4 URDF file!
    try:
        robot_id = p.loadURDF(urdf_path, [0,0,0], useFixedBase=True)
    except p.error:
        print(f"❌ CRITICAL ERROR: Cannot find URDF file at '{urdf_path}'!")
        exit()
        
    n_joints = p.getNumJoints(robot_id)
    print(f"PyBullet: {n_joints} joints loaded")
    return robot_id, list(range(n_joints))



# === HAND TRACKING ===
def hand_tracking_process(xy_queue):
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    while cap.isOpened() and not stop_event.is_set():
        ret, frame = cap.read()
        if not ret: continue
        
        # Undistort the frame for accurate hand tracking
        dst = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

        # Mirror so left hand movement = left on screen
        if MIRROR_CAMERA:
            dst = cv2.flip(dst, 1)
        
        rgb = cv2.cvtColor(dst, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)
        
        display = dst.copy()
        if results.multi_hand_landmarks:
            for lm in results.multi_hand_landmarks:
                mp.solutions.drawing_utils.draw_landmarks(display, lm, 
                    mp.solutions.hands.HAND_CONNECTIONS)
                
                # --- GRAB DETECTION ---
                # Check if fingertips are lower than knuckles (y-axis goes down in pixels)
                fingers_up = 0
                if lm.landmark[8].y < lm.landmark[6].y: fingers_up += 1  # Index finger
                if lm.landmark[12].y < lm.landmark[10].y: fingers_up += 1 # Middle finger
                if lm.landmark[16].y < lm.landmark[14].y: fingers_up += 1 # Ring finger
                if lm.landmark[20].y < lm.landmark[18].y: fingers_up += 1 # Pinky
                
                # If less than 2 fingers are up, we see it as a fist (Grab = 1)
                grab_state = 1 if fingers_up < 2 else 0
                
                # --- POSITION CALCULATION ---
                wrist = lm.landmark[0]
                wrist_px_x = int(wrist.x * 640)
                wrist_px_y = int(wrist.y * 480)
                
                wrist_mm_x = wrist_px_x * MM_PER_PX  # horizontal (left/right)
                wrist_mm_y = wrist_px_y * MM_PER_PX  # vertical   (up/down)
                
                # Add X, Y and the Grab Status to the queue
                if xy_queue.full():
                    try: xy_queue.get_nowait()
                    except: pass
                xy_queue.put((wrist_mm_x, wrist_mm_y, grab_state))
                
                # --- VISUAL FEEDBACK (visible in Unity, no separate window anymore) ---
                cv2.circle(display, (wrist_px_x, wrist_px_y), 15, (0,255,0), -1)
                cv2.putText(display, f"ROBOT: {wrist_mm_x:.0f}mm {wrist_mm_y:.0f}mm", 
                           (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                
                status_text = "GRAB: CLOSED" if grab_state == 1 else "GRAB: OPEN"
                color = (0, 0, 255) if grab_state == 1 else (255, 0, 0)
                cv2.putText(display, status_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)
        
        # Send frame to Unity camera stream
        if frame_queue.full():
            try: frame_queue.get_nowait()
            except: pass
        frame_queue.put_nowait(display.copy())

        # cv2.imshow has been removed — feed now only runs via Unity
    
    cap.release()



# === PYBULLET IK CONTROL ===
def robot_control_process(xy_queue, client, robot_id, joint_indices):
    print("🤖 PyBullet IK → Unity DEGREES")
    
    while True:
        try:
            # Now grabs THREE values from the queue (including grab_state)
            wrist_mm_x, wrist_mm_y, grab_state = xy_queue.get(timeout=0.1)
            
            # Hand → workspace mapping
            # Camera X (horizontal) → robot Y (left/right)
            # Camera Y (vertical)   → robot X (forward/backward)
            # Swap the two lines if the axes still feel wrong
            target_pos = [
                WORKSPACE_X_OFFSET - wrist_mm_y * MOTION_SCALE,  # camera vertical   → robot X
                WORKSPACE_Y_OFFSET + wrist_mm_x * MOTION_SCALE,  # camera horizontal → robot Y
                WORKSPACE_Z
            ]
            
            # PyBullet IK for the Faze4
            joint_pos = p.calculateInverseKinematics(
                robot_id, 
                endEffectorLinkIndex=6, 
                targetPosition=target_pos,
                maxNumIterations=50
            )
            
            # Convert to degrees for Unity
            q_deg = np.degrees(joint_pos[:6])
            
            # Send direct to Unity
            func_data(client, q_deg)
            func_grab(client, grab_state) 
            
        except Empty: continue
        except Exception as e:
            print(f"PyBullet Error: {e}")



# === MAIN ===
print("🚀 PYBULLET FAZE4 HAND CONTROL")
print(f"   Motion scale:      {MOTION_SCALE}")
print(f"   Mirror camera: {MIRROR_CAMERA}")


client = tcp_init()
robot_id, joint_indices = init_pybullet_robot()


p_robot = threading.Thread(target=robot_control_process, 
                          args=(xy_queue, client, robot_id, joint_indices), daemon=True)
p_robot.start()


threading.Thread(target=camera_stream_server, daemon=True).start()


try:
    hand_tracking_process(xy_queue)
except KeyboardInterrupt:
    print("🛑 Stopped")
finally:
    stop_event.set()
    p.disconnect()
    client.close()
    cv2.destroyAllWindows()