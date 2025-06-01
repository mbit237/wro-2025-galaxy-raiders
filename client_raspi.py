import socket
import pickle 

HEADER = 42 
PORT = 5050
FORMAT = 'utf-8'

DISCONNECT_MESSAGE = "!DISCONNECT"
SERVER = '192.168.8.189'
ADDR = (SERVER, PORT)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
client.connect(ADDR)
arr = []
for x in range(400):
    arr.append([x/400*360, x*10])

def send(msg):
    message = pickle.dumps(msg)
    msg_length = len(message)
    l1 = msg_length >> 8
    l2 = msg_length & 0xFF
    header = bytes([HEADER, l1, l2])
    message = header + message
    client.send(message)

send(arr)
send(DISCONNECT_MESSAGE)
