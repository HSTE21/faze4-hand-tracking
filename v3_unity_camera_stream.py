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


# === CALIBRATION ===
MM_PER_PX = 0.4085
mtx = np.array([[2.00313622e+03, 0.00000000e+00, 1.13719259e+03],
                [0.00000000e+00, 2.00602079e+03, 7.34877818e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.array([[-2.57939135e-02,  3.50765930e-01, -1.66865917e-04, -7.56033165e-04,
                  -8.65799370e-01]])
newcameramtx = np.array([[1.93156188e+03, 0.00000000e+00, 1.13553543e+03],
                         [0.00000000e+00, 1.94087193e+03, 7.31467517e+02],
                         [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
roi = (39, 22, 2190, 1460)
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
xy_queue = Queue(maxsize=1)
frame_queue = Queue(maxsize=1)         


# === UNITY TCP ===
def tcp_init(host='127.0.0.1', port=55001):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print(f"✅ Connected Unity {host}:{port}")
    return sock


def func_data(client, q_deg):
    q_deg = np.clip(q_deg, -170, 170)
    msg = f"ModRobot:{','.join(f'{qi:.3f}' for qi in q_deg)}"
    data = bytes([len(msg)]) + msg.encode('utf-8')
    client.send(data)


def func_grab(client, state):
    msg = f"ModGrab:{int(state)}"
    data = bytes([len(msg)]) + msg.encode('utf-8')
    client.send(data)



def camera_stream_server(port=55002):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', port))
    server.listen(1)
    print(f"📷 Waiting for Unity camera connection on port {port}...")
    conn, addr = server.accept()
    print(f"📷 Unity camera connected: {addr}")
    try:
        while True:
            if not frame_queue.empty():
                frame = frame_queue.get_nowait()
                ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                if ret:
                    data = buf.tobytes()
                    conn.sendall(struct.pack('>I', len(data)) + data)
    except Exception as e:
        print(f"📷 Camera connection lost: {e}")
    finally:
        conn.close()
        server.close()


# === PYBULLET ROBOT (Faze4) ===
def init_pybullet_robot():
    p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    # Construct an absolute path to the URDF file to prevent "file not found" errors.
    # This works even if the script is run from a different working directory.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "faze4.urdf")
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
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: continue
        
        dst = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(dst, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)
        
        display = dst.copy()
        if results.multi_hand_landmarks:
            for lm in results.multi_hand_landmarks:
                mp.solutions.drawing_utils.draw_landmarks(display, lm, 
                    mp.solutions.hands.HAND_CONNECTIONS)
                
                fingers_up = 0
                if lm.landmark[8].y < lm.landmark[6].y: fingers_up += 1
                if lm.landmark[12].y < lm.landmark[10].y: fingers_up += 1
                if lm.landmark[16].y < lm.landmark[14].y: fingers_up += 1
                if lm.landmark[20].y < lm.landmark[18].y: fingers_up += 1
                
                grab_state = 1 if fingers_up < 2 else 0
                
                wrist = lm.landmark[0]
                wrist_px_x = int(wrist.x * 640)
                wrist_px_y = int(wrist.y * 480)
                wrist_mm_x = wrist_px_x * MM_PER_PX
                wrist_mm_y = wrist_px_y * MM_PER_PX
                
                if xy_queue.full():
                    try: xy_queue.get_nowait()
                    except: pass
                xy_queue.put((wrist_mm_x, wrist_mm_y, grab_state))
                
                cv2.circle(display, (wrist_px_x, wrist_px_y), 15, (0,255,0), -1)
                cv2.putText(display, f"ROBOT: {wrist_mm_x:.0f}mm {wrist_mm_y:.0f}mm", 
                           (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                status_text = "GRAB: CLOSED" if grab_state == 1 else "GRAB: OPEN"
                color = (0, 0, 255) if grab_state == 1 else (255, 0, 0)
                cv2.putText(display, status_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)
        
        
        if frame_queue.full():
            try: frame_queue.get_nowait()
            except: pass
        frame_queue.put_nowait(display.copy())

        cv2.imshow('🤖 HAND → FAZE4 UNITY (q=quit)', display)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
    
    cap.release()
    cv2.destroyAllWindows()


# === PYBULLET IK CONTROL ===
def robot_control_process(xy_queue, client, robot_id, joint_indices):
    print("🤖 PyBullet IK → Unity DEGREES")
    while True:
        try:
            wrist_mm_x, wrist_mm_y, grab_state = xy_queue.get(timeout=0.1)
            scale = 0.0008
            target_pos = [
                0.35 + wrist_mm_x * scale,
                -0.25 + wrist_mm_y * scale,
                0.197
            ]
            joint_pos = p.calculateInverseKinematics(
                robot_id, 
                endEffectorLinkIndex=6, 
                targetPosition=target_pos,
                maxNumIterations=50
            )
            q_deg = np.degrees(joint_pos[:6])
            func_data(client, q_deg)
            func_grab(client, grab_state)
        except Empty: continue
        except Exception as e:
            print(f"PyBullet Error: {e}")


# === MAIN ===
print("🚀 PYBULLET FAZE4 HAND CONTROL")

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

p.disconnect()
client.close()
cv2.destroyAllWindows()
