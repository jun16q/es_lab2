import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import socket
import threading

# --- 數據緩衝區 ---
max_points = 200
data_ax, data_ay, data_az = [], [], []

# --- TCP Server 設定 ---
HOST = "0.0.0.0"  # 綁定所有網卡，STM32 連此 IP
PORT = 5000       # 你選的 port

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)
print(f"Server listening on {HOST}:{PORT}")

def handle_client(conn):
    global data_ax, data_ay, data_az
    buffer = ""
    while True:
        try:
            data = conn.recv(1024).decode('utf-8')
            if not data:
                break
            buffer += data
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                try:
                    ax_val, ay_val, az_val = map(float, line.strip().split(","))
                    ax_val /= 1000
                    ay_val /= 1000
                    az_val /= 1000
                    data_ax.append(ax_val)
                    data_ay.append(ay_val)
                    data_az.append(az_val)
                    # 保留最近 max_points 筆
                    if len(data_ax) > max_points:
                        data_ax.pop(0)
                        data_ay.pop(0)
                        data_az.pop(0)
                    # 印出剛接收到的資料
                    print(f"{ax_val:.2f}, {ay_val:.2f}, {az_val:.2f}")
                except ValueError:
                    continue
        except Exception as e:
            print("Connection closed:", e)
            break
    conn.close()

# 等待 STM32 client 連線
conn, addr = server.accept()
print("Connected by", addr)

# 用 thread 處理 client
threading.Thread(target=handle_client, args=(conn,), daemon=True).start()

# --- GUI ---
root = tk.Tk()
root.title("3-Axis Acceleration (TCP Server)")

fig, ax = plt.subplots()
ax.set_title("Acceleration over Time")
ax.set_xlabel("Sample")
ax.set_ylabel("Acceleration (g)")
ax.set_ylim(0, max_points)
ax.set_ylim(-2, 2)
ax.grid(True)

line_ax, = ax.plot([], [], label="Ax", color='r')
line_ay, = ax.plot([], [], label="Ay", color='g')
line_az, = ax.plot([], [], label="Az", color='b')
ax.legend()

# 更新函數
def update(frame):

    x = range(len(data_ax))
    line_ax.set_data(x, data_ax)
    line_ay.set_data(x, data_ay)
    line_az.set_data(x, data_az)
    ax.set_xlim(max(0, len(data_ax)-max_points), len(data_ax))
    return line_ax, line_ay, line_az


canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# ani = animation.FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
# 將 blit 改為 False
ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)


root.mainloop()
