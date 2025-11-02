import socket
import pickle 
import json
import time

FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
client = None

def connect():
    global client
    PORT = 5050
    SERVER = 'LAPTOP-AIQ4T307'
    ADDR = (SERVER, PORT)
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.setsockopt(sb ocket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    client.connect(ADDR)
    print("connected")

def send(msg):
    HEADER = 42 
    message = pickle.dumps(msg)
    msg_length = len(message)
    l1 = msg_length >> 8
    l2 = msg_length & 0xFF
    header = bytes([HEADER, l1, l2])
    message = header + message
    # print(message)
    print("before sent")
    client.send(message)
    print("sent")

# connect()
# with open("records.txt") as f:
#     for x in range(50):
#         send(json.loads(f.readline()))
#         time.sleep(0.1)
# send(DISCONNECT_MESSAGE)