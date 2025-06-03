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

def draw(readings):
    print(readings)
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    for reading in readings:
        ax.plot([reading[0] * math.pi / 180], [reading[1]], marker='o', markersize=5)

    ax.set_title("Lidar Points")
    plt.show()

print("[STARTING] server is starting...")
start()
