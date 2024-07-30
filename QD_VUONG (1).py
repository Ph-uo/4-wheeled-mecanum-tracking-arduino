import asyncio
import websockets
import json
import cv2
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
import pandas as pd
import threading
import time
import math

# WebSocket variables
ws_url = "ws://192.168.100.12/ws"
excel_file_path = r'C:\Users\Admin\Documents\Arduino\temp_nckh\data.xlsx'

x = 0
y = 0
theta = 0
ex = 0
ey = 0
eth = 0
thetad = 3.14/2
heso=280
xg = 0  # Green object x-coordinate
yg = 0  # Green object y-coordinate
websocket = None
esp32_connected = True
kp_ex, ki_ex, kd_ex = 0.08, 0.001, 0.0001
kp_ey, ki_ey, kd_ey = 0.08, 0.001, 0.0001
kp_eth, ki_eth, kd_eth = 0.08, 0.001, 0.0001
error_ex, error_prevex = 0, 0
integ_ex, integ_prevex = 0, 0
error_ey, error_prevey = 0, 0
integ_ey, integ_prevey = 0, 0
error_eth, error_preveth = 0, 0
integ_eth, integ_preveth = 0, 0
ex1, ey1, eth1 = 0, 0, 0
dt = 0.1
lx = 0.225
ly = 0.18
a = 0
pi = 3.14
r = 0.075

# Frame dimensions
frame_width = 1260
frame_height = 975
center_x_frame = frame_width // 2
center_y_frame = frame_height // 2

# Frame dimensions
frame_width1 = 1260
frame_height1 = 975
center_x_frame1 = frame_width1 // 2
center_y_frame1 = frame_height1 // 2

# Initialize camera
cap = cv2.VideoCapture(0)

# Variables for plotting
trajectory_data = deque(maxlen=2500)
error_data = deque(maxlen=3000)
rpm_data = deque(maxlen=3000)
start_time = time.time()

# Tkinter setup
root = tk.Tk()
root.title('4 wheel mecanum mobile robot trajectory')
root.geometry("1900x1100")
camera_label = tk.Label(root)
camera_label.place(x=0, y=0, width=frame_width, height=frame_height)

# Plot setup
fig2, ax2 = plt.subplots()
canvas2 = FigureCanvasTkAgg(fig2, master=root)
canvas_widget2 = canvas2.get_tk_widget()
canvas_widget2.place(x=frame_width, y=-25, width=650, height=400)

fig3, ax3 = plt.subplots()
canvas3 = FigureCanvasTkAgg(fig3, master=root)
canvas_widget3 = canvas3.get_tk_widget()
canvas_widget3.place(x=frame_width, y=375, width=650, height=250)

fig4, ax4 = plt.subplots()
canvas4 = FigureCanvasTkAgg(fig4, master=root)
canvas_widget4 = canvas4.get_tk_widget()
canvas_widget4.place(x=frame_width, y=625, width=650, height=350)

def init_data(i):
    global global_dataframes
    global_dataframes = [pd.DataFrame() for _ in range(i)]

def get_data(*args):
    global global_dataframes
    for idx, arg in enumerate(args):
        global_dataframes[idx] = global_dataframes[idx]._append(pd.DataFrame([arg]), ignore_index=True)
    time.sleep(0.1)

def save_data(excel_file_path):
    combined_df = pd.concat(global_dataframes, axis=1)
    combined_df.columns = [f'Column_{i+1}' for i in range(len(global_dataframes))]  # Dynamic column naming
    with pd.ExcelWriter(excel_file_path) as writer:
        combined_df.to_excel(writer, index=False)
        print('Save success')

def compute_center_coordinates(x, y, w, h):
    center_x = x + w // 2
    center_y = y + h // 2
    return center_x, center_y

def trajectory(t):
    side_length = 2.0
    period = 40.0
    t_normalized = t % period
    segment_time = period / 4
    if t_normalized < segment_time:
        xd = 1
        yd = -1 + (t_normalized / segment_time) * side_length
        xd_dot = 0
        yd_dot = side_length / segment_time
        thetad_dot = math.atan2(yd_dot, xd_dot)
    elif t_normalized < 2 * segment_time:
        xd = 1 - ((t_normalized - segment_time) / segment_time) * side_length
        yd = 1
        xd_dot = -side_length / segment_time
        yd_dot = 0
        thetad_dot = math.atan2(yd_dot, xd_dot)
    elif t_normalized < 3 * segment_time:
        xd = -1
        yd = 1 - ((t_normalized - 2 * segment_time) / segment_time) * side_length
        xd_dot = 0
        yd_dot = -side_length / segment_time
        thetad_dot = math.atan2(yd_dot, xd_dot)
    else:
        xd = -1 + ((t_normalized - 3 * segment_time) / segment_time) * side_length
        yd = -1
        xd_dot = side_length / segment_time
        yd_dot = 0
        thetad_dot = math.atan2(yd_dot, xd_dot)
    return xd, yd, xd_dot, yd_dot, thetad_dot

def trajectory2():
    vertices = [
        (1, -1),
        (1, 1),
        (-1, 1),
        (-1, -1),
        (1, -1)
    ]
    return vertices

def moving_average(data, window_size):
    if len(data) < window_size:
        return data
    return np.convolve(data, np.ones(window_size) / window_size, mode='valid')
def find_closest_point(x, y, reference_trajectory):
    closest_point = min(reference_trajectory, key=lambda point: math.sqrt((point[0] - x) ** 2 + (point[1] - y) ** 2))
    return closest_point

def calculate_errors(x, y):
    global ex, ey, eth, xd, yd, a
    global integ_prevex, error_prevex, integ_prevey, error_prevey, integ_preveth, error_preveth
    global Ui, VXE, Ui1, Ui2, Ui3, Ui4, RPM1, RPM2, RPM3, RPM4
    a = lx + ly
    current_time = time.time() - start_time
    xd, yd, xd_dot, yd_dot, thetad_dot = trajectory(current_time)
    square_trajectory = [trajectory(t) for t in np.linspace(0, current_time, 100)]
    closest_xd, closest_yd, _, _, _ = find_closest_point(x, y, square_trajectory)
    closest_xd = round((closest_xd), 2)
    closest_yd = round((closest_yd), 2)
    ex = 100 * round(x - closest_xd, 2)
    ey = 100 * round(y - closest_yd, 2)
    eth = round(thetad - theta, 2)
    error_ex = ex
    integ_ex = integ_prevex + (dt * (error_ex + error_prevex) / 2)
    ex1 = kp_ex * error_ex + ki_ex * integ_ex + (kd_ex * ((error_ex - error_prevex) / dt))
    integ_prevex = integ_ex
    error_prevex = error_ex
    error_ey = ey
    integ_ey = integ_prevey + (dt * (error_ey + error_prevey) / 2)
    ey1 = kp_ey * error_ey + ki_ey * integ_ey + (kd_ey * ((error_ey - error_prevey) / dt))
    integ_prevey = integ_ey
    error_prevey = error_ey
    error_eth = eth
    integ_eth = integ_preveth + (dt * (error_eth + error_preveth) / 2)
    eth1 = kp_eth * error_eth + ki_eth * integ_eth + (kd_eth * ((error_eth - error_preveth) / dt))
    integ_preveth = integ_eth
    error_preveth = error_eth

    vxa = ex1
    vya = ey1
    w1 = eth1
    vx1 = (math.cos(theta) * vxa) + (math.sin(theta) * vya)
    vy1 = (-math.sin(theta) * vxa) + (math.cos(theta) * vya)
    ros = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])
    P_dot = np.array([xd_dot, yd_dot, thetad_dot])
    H_inv = np.linalg.inv(ros)
    V = np.dot(H_inv, P_dot)
    vx, vy, w = V
    H = (1 / r) * np.array([
        [1, -1, -a],
        [1, 1, a],
        [1, 1, -a],
        [1, -1, a]
    ])
    V = np.array([vx, vy, w])
    Ui = np.dot(H, V)
    VXE = math.sqrt(vx ** 2 + vy ** 2)

    Ui1 = ((1 / r) * vx1) + ((-1 / r) * vy1) + ((-a / r) * w1)
    Ui2 = ((1 / r) * vx1) + ((1 / r) * vy1) + ((a / r) * w1)
    Ui3 = ((1 / r) * vx1) + ((1 / r) * vy1) + ((-a / r) * w1)
    Ui4 = ((1 / r) * vx1) + ((-1 / r) * vy1) + ((a / r) * w1)
    RPM1 = ((Ui1 * r) / (pi * 2 * r)) * 60
    RPM2 = ((Ui2 * r) / (pi * 2 * r)) * 60
    RPM3 = ((Ui3 * r) / (pi * 2 * r)) * 60
    RPM4 = ((Ui4 * r) / (pi * 2 * r)) * 60
    # print(RPM1, RPM2, RPM3, RPM4)
    print(VXE)
    rpm_data.append((current_time, VXE, w))
    error_data.append((current_time, ex, ey, eth))
    print(f"Điểm đặt ({closest_xd}, {closest_yd}), Camera ({x}, {y}): ex={ex}, ey={ey}, eth={eth}")
    get_data(current_time,xd,yd,thetad,x,y,theta,ex,ey,eth,vx,vy,w)

def calculate_angle():  # Tính góc theta giữa xe với trục x
    global xg, yg, x, y
    dx = xg - x
    dy = yg - y
    theta = math.atan2(dy, dx)  # rad
    theta = round(theta, 2)
    if theta<0:
        theta=theta+2*3.14
    return theta

def detect_and_plot():
    global x, y, xg, yg, theta, esp32_connected, ex, ey, eth
    global RPM1, RPM2, RPM3, RPM4, VXE, w
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        root.after(10, detect_and_plot)
        return
    frame = cv2.resize(frame, (frame_width, frame_height))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([90, 130, 130])
    upper_blue = np.array([130, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    def get_scale_for_region(cx, cy):
        if cx < center_x_frame / 2 and cy < center_y_frame / 2:
            return 297  # Top-left region
        elif cx < center_x_frame and cy < center_y_frame / 2:
            return 285  # Top-center region
        elif cx < center_x_frame * 1.5 and cy < center_y_frame / 2:
            return 286  # Top-right region
        elif cx < center_x_frame / 2 and cy < center_y_frame:
            return 295  # Middle-left region
        elif cx < center_x_frame and cy < center_y_frame:
            return 285  # Middle-center region
        elif cx < center_x_frame * 1.5 and cy < center_y_frame:
            return 295  # Middle-right region
        elif cx < center_x_frame / 2 and center_y_frame <= cy < center_y_frame * 1.25:
            return 296
        elif cx < center_x_frame / 2 and center_y_frame <= cy < center_y_frame * 1.25:
            return 295.7
        elif cx < center_x_frame / 2 and cy < center_y_frame * 1.5:
            return 296  # Bottom-left region
        elif cx < center_x_frame and cy < center_y_frame * 1.5:
            return 290  # Bottom-center region
        else:
            return 291  # Bottom-right region

    if len(contours_blue) > 0:
        for contour in contours_blue:
            area = cv2.contourArea(contour)
            if area > 100:
                x, y, w, h = cv2.boundingRect(contour)
                center_x, center_y = compute_center_coordinates(x, y, w, h)
                scale = get_scale_for_region(center_x, center_y)
                adjusted_x = (center_x - center_x_frame) / scale
                adjusted_y = (center_y_frame - center_y) / scale
                adjusted_x = round(adjusted_x, 2)
                adjusted_y = round(adjusted_y, 2)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f'Blue X: {adjusted_x}, Y: {adjusted_y}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2)
                trajectory_data.append((adjusted_x, adjusted_y))
                x = adjusted_x
                y = adjusted_y
                calculate_errors(x, y)

        for contour in contours_yellow:
            area1 = cv2.contourArea(contour)
            if area1 > 100:
                xg, yg, w1, h1 = cv2.boundingRect(contour)
                center_gx, center_gy = compute_center_coordinates(xg, yg, w1, h1)
                scale = get_scale_for_region(center_gx, center_gy)
                adjusted_gx = (center_gx - center_x_frame) / scale
                adjusted_gy = (center_y_frame - center_gy) / scale
                adjusted_gx = round(adjusted_gx, 2)
                adjusted_gy = round(adjusted_gy, 2)
                cv2.rectangle(frame, (xg, yg), (xg + w1, yg + h1), (0, 255, 0), 2)
                xg = adjusted_gx
                yg = adjusted_gy

        theta = calculate_angle()
        cv2.putText(frame, f'theta: {theta:.2f}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.line(frame, (center_x_frame1, 0), (center_x_frame1, 1000), (255, 0, 0), 2)
        cv2.line(frame, (0, center_y_frame1), (2000, center_y_frame1), (255, 0, 0), 2)

    else:
        x = 0
        y = 0
        ex = 0
        ey = 0
        eth = 0
        VXE = 0
        w = 0
        RPM1 = 0
        RPM2 = 0
        RPM3 = 0
        RPM4 = 0
        current_time = time.time() - start_time
        rpm_data.append((current_time, VXE, w))
        error_data.append((current_time, ex, ey, eth))

    if esp32_connected:
        ax2.clear()
        traj_x_data = [data[0] for data in trajectory_data]
        traj_y_data = [data[1] for data in trajectory_data]
        ax2.set_title('Robot Trajectory')
        square_trajectory2 = trajectory2()
        square_x_data2, square_y_data2 = zip(*square_trajectory2)
        ax2.plot(square_x_data2, square_y_data2, color='red', linewidth=3, label='Reference')
        if traj_x_data and traj_y_data:
            ax2.plot(traj_x_data, traj_y_data, color='blue', linestyle='--', linewidth=2.3, label='Robot Trajectory')
            ax2.set_xlabel('X coordinate(m)')
            ax2.set_ylabel('Y coordinate(m)')
        ax2.axhline(y=0, color='black', linestyle='--')
        ax2.axvline(x=0, color='black', linestyle='--')
        ax2.legend(loc='upper right')  # Pin label to the top right
        canvas2.draw()

        ax3.clear()
        time_data = [data[0] for data in error_data]
        ex_data = [data[1] for data in error_data]
        ey_data = [data[2] for data in error_data]
        eth_data = [data[3] for data in error_data]
        window_size = 10
        smoothed_ex_data = moving_average(ex_data, window_size)
        smoothed_ey_data = moving_average(ey_data, window_size)
        smoothed_eth_data = moving_average(eth_data, window_size)
        smoothed_time_data = moving_average(time_data, window_size)
        ax3.set_title('Error over Time')
        ax3.plot(smoothed_time_data, smoothed_ex_data, color='red', linestyle='--', linewidth=2.5, label='Error ex (cm)')
        ax3.plot(smoothed_time_data, smoothed_ey_data, color='blue', linestyle=':', linewidth=2.5, label='Error ey (cm)')
        ax3.plot(smoothed_time_data, smoothed_eth_data, color='yellow', linewidth=2.5, label='Error eth (rad/s)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Error Value')
        ax3.set_ylim(-40, 40)
        ax3.legend(loc='upper right')  # Pin label to the top right
        canvas3.draw()

        ax4.clear()
        time1_data = [data[0] for data in rpm_data]
        VXE_data = [data[1] for data in rpm_data]
        w_data = [data[2] for data in rpm_data]
        window_size1 = 5
        smoothed_time1_data = moving_average(time1_data, window_size1)
        smoothed_VXE_data = moving_average(VXE_data, window_size1)
        smoothed_w_data = moving_average(w_data, window_size1)
        ax4.set_title('V and W over time')
        ax4.plot(smoothed_time1_data, smoothed_VXE_data, color='black', linestyle='--', linewidth=2.5, label='V (m/s)')
        ax4.plot(smoothed_time1_data, smoothed_w_data, color='red', linestyle=':', linewidth=2.5, label='w (rad/s)')
        ax4.set_ylabel('Value')
        ax4.set_ylim(-1, 1)
        ax4.legend(loc='upper right')  # Pin label to the top right
        canvas4.draw()

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(frame)
    img_tk = ImageTk.PhotoImage(image=img)
    camera_label.imgtk = img_tk
    camera_label.config(image=img_tk)
    camera_label.after(10, detect_and_plot)

async def send_data():
    global x, y, websocket, esp32_connected
    while True:
        try:
            print('đang cố gắng kết nối với esp32')
            websocket = await websockets.connect(ws_url, ping_interval=10, ping_timeout=5)
            esp32_connected = True
            print("Connected to the WebSocket server")
            while True:
                data2send = {"x": x, "y": y, "theta": theta}
                await websocket.send(json.dumps(data2send))
                print("Sent:", "x:", x, "y:", y, "theta: ", theta)
                await asyncio.sleep(0.1)
        except websockets.exceptions.ConnectionClosed as e:
            print(f"Connection closed with exception: {e}")
            esp32_connected = True
        except asyncio.TimeoutError:
            print("Connection attempt timed out")
            esp32_connected = True
        finally:
            if websocket:
                await websocket.close()
                print ('websocket end')
                break
            # reconnect=input ('reconnect? y to reconnect ')
            # if reconnect=='y'or reconnect=='Y':
            #     print("Reconnecting in 3 seconds...")
            #     await asyncio.sleep(3)
            # else:
            #     break

def run_websocket():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    tasks = [loop.create_task(send_data())]
    loop.run_until_complete(asyncio.wait(tasks))

def run_tkinter():
    detect_and_plot()
    #root.mainloop()
    root.protocol("WM_DELETE_WINDOW", root.quit)
    root.mainloop()
    root.destroy()

init_data(13)#x,y,th,ex,ey,eth,vx,vy,w
threading.Thread(target=run_websocket, daemon=True).start()
run_tkinter()
cap.release()
cv2.destroyAllWindows()
save_data(excel_file_path)
#esp32_connected= False