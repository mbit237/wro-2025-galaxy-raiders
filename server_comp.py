import socket 
import pickle
import matplotlib.pyplot as plt
import math

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
server.bind(ADDR)

def start():
    server.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")

    conn, addr = server.accept() 

    print(f"[NEW CONNECTION] {addr} connected.")
    connected = True
    ptr = 0 
    msg_length = 0 
    while connected:
        chars = conn.recv(10000)
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
                        connected = False
                    else:
                        draw(message)
        

    conn.close()
    print("Connection closed")

    # server.shutdown(socket.SHUT_RD)
    # server.close()

# Polar
# fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

# Cartesian
fig, ax = plt.subplots()
ax.grid(True)
ax.set_xlim(-100, 3100)
ax.set_ylim(-100, 3100)

def draw(readings):
    draw_cartesian(readings)
    # draw_polar(readings)

def draw_cartesian(readings):
    x = []
    y = []
    for reading in readings:
        x.append(reading[0])
        y.append(reading[1])

    lx = [1000, 1000, 2000, 2000, 0, 0, 3000, 3000]
    ly = [1000, 2000, 1000, 2000, 0, 3000, 0, 3000]
    ax.clear()
    ax.scatter(x, y, marker="o")
    ax.scatter(lx, ly, marker="x")
    plt.show(block=False)
    fig.canvas.draw_idle()
    fig.canvas.flush_events()

def draw_polar(readings):
    ax.clear()
    for reading in readings:
        print(reading)
        # update_plt([reading[0] * math.pi / 180], [reading[1]]) # , marker='o', markersize=5
        ax.plot((reading[0] + 90) * math.pi / 180, reading[1], marker='o', markersize=5)
    # plt.show()
    plt.show(block=False)
    fig.canvas.draw_idle()
    fig.canvas.flush_events()

print("[STARTING] server is starting...")
start()
