import tkinter as tk
import math
import socket 
import pickle
# import matplotlib.pyplot as plt

# intialise socket 

def get_lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return str(s.getsockname()[0])
    finally:
        s.close()

print("My LAN IP is:", get_lan_ip())

HEADER = 42  
HEADER_LENGTH = 3
PORT = 5050
SERVER = get_lan_ip() 
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
buf = bytearray(10000)
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
# Disable Nagle's algorithm by setting TCP_NODELAY to 1
server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
server.setblocking(False)
server.bind(ADDR)
server.listen()
print(f"[LISTENING] Server is listening on {SERVER}")

conn = None
buf = bytearray(10000)
ptr = 0
msg_length = 0

SIZE = 900
SCALE = SIZE / 3000
ROBOT_R = 15
ARROW_LEN = 100

root = tk.Tk()
root.geometry("1600x1000")

history_data = []

# Draw functions
def circle(x, y, r=5, fill="cyan", outline="black", width=1):
    x = x * SCALE
    y = 900 - y * SCALE
    field.create_oval(x-r,y-r,x+r,y+r, fill=fill, outline=outline, width=width)

def line(x1, y1, x2, y2, fill="blue", arrow=tk.LAST): 
    x1 = x1 * SCALE
    y1 = 900 - y1 * SCALE
    x2 = x2 * SCALE
    y2 = 900 - y2 * SCALE
    field.create_line(x1, y1, x2, y2, arrow=arrow, fill=fill)

def draw_robot(idx):
    field.create_image((0, 0), image=field_img, anchor=tk.NW)
    spike_pose, pose, match_spikes, index, misc = history_data[idx]
    x = pose[0]
    y = pose[1]
    heading = pose[2]
    circle(x, y, r=ROBOT_R, fill="yellow")
    a = heading / 180 * math.pi
    x2 = math.cos(a) * ARROW_LEN + x
    y2 = math.sin(a) * ARROW_LEN + y 
    line(x, y, x2, y2)

    x_spike = spike_pose[0]
    y_spike = spike_pose[1]
    heading_spike = spike_pose[2]
    a_spike = heading_spike / 180 * math.pi
    x2_spike = math.cos(a_spike) * ARROW_LEN + x_spike
    y2_spike = math.sin(a_spike) * ARROW_LEN + y_spike 
    line(x_spike, y_spike, x2_spike, y2_spike, fill="red")
    draw_trail(idx)
    draw_trail_spikes(idx)
    for match_spike in match_spikes:
        circle(match_spike[0], match_spike[1])
    update_data_label(idx)
    
def draw_trail(idx):
    end = 0
    if len(history_data) > 50:
        end = len(history_data) - 50
    for i in range(idx, end, -1):
        pose = history_data[i][1]
        x = pose[0]
        y = pose[1]
        pose = history_data[i-1][1]
        x2 = pose[0]
        y2 = pose[1]
        line(x, y, x2, y2, fill="magenta", arrow=None)

def draw_trail_spikes(idx):
    end = 0
    if len(history_data) > 50:
        end = len(history_data) - 50
    for i in range(idx, end, -1):
        pose = history_data[i][0]
        x = pose[0]
        y = pose[1]
        pose = history_data[i-1][0]
        x2 = pose[0]
        y2 = pose[1]
        line(x, y, x2, y2, fill="green", arrow=None)

def scale_change(v):
    draw_robot(int(v))

def update_data_label(idx):
    global data 
    pose, spike_pose, match_spikes, index, misc = history_data[idx]
    data["text"] = "pose: " + str(pose) + '\n'
    data["text"] += "spike_pose: " + str(spike_pose) + '\n'
    data["text"] += "match_spikes: " + '\n'
    for match_spike in match_spikes:
        data["text"] += "\t" + str(match_spike) + '\n'
    data["text"] += "idx: " + str(index) + '\n'
    data["text"] += "misc: " + str(misc)

# widgets 

v1 = tk.IntVar()

slider = tk.Scale(root, command=scale_change, variable=v1, from_=0, to=len(history_data)-1, orient=tk.HORIZONTAL, length=900)
# slider.pack()
slider.grid(column=0, row=0)

field = tk.Canvas(root, width=900, height=900)
# field.pack()
field.grid(column=0, row=1)
field_img = tk.PhotoImage(file="FutureEngineers_Playfield_900.png")

data = tk.Label(root, text="Waiting for data", wraplength=500, justify=tk.LEFT)
data.grid(column=1, row=0, rowspan=2, padx=10, pady=10, sticky="ew") # sticky="ew" allows the label to expand horizontally when window is resized
# ew -- eastwest

def socket_connect():
    global conn, ptr, msg_length
    if conn is None: # no current connection 
        try:
            conn, addr = server.accept() 
            print(f"[NEW CONNECTION] {addr} connected.")
            conn.setblocking(False) # set connection to non-blocking
        except BlockingIOError: # nothing received
            pass 
    else: # current connection 
        try:
            chars = conn.recv(10000) 
            if not chars: # checks for empty bytes case (client cleanly disconnects)
                print("[DISCONNECTED]")
                conn.close()
                conn = None
                return
            for char in chars:
                if ptr == 0:
                    if char == HEADER:
                        buf[0] = char
                        ptr += 1
                else:
                    buf[ptr] = char
                    ptr += 1
                    if ptr == HEADER_LENGTH:
                        msg_length = buf[1] << 8 | buf[2]
                    elif ptr == HEADER_LENGTH + msg_length:
                        ptr = 0 
                        message = pickle.loads(buf[3:])
                        if message == DISCONNECT_MESSAGE:
                            print("[CLIENT DISCONNECT]")
                            conn.close()
                            conn = None
                        else:
                            # print(message, type(message))
                            spike_pose, merged_pose, match_spikes, idx, misc = message
                            print(history_data)
                            history_data.append([spike_pose, merged_pose, match_spikes, idx, misc])

                            slider.config(to=len(history_data)-1)

                            v1.set(len(history_data)-1)
                            draw_robot(len(history_data)-1)
                            
        except BlockingIOError: # no data yet 
            pass
        except ConnectionResetError: # client forcefully closed connection 
            print("Client Reset")
            conn = None 

    root.after(1000, socket_connect)

root.after(1000, socket_connect)

root.mainloop() # run application once it is ready -- blocks