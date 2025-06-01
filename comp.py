import socket 
import threading
import pickle 

def get_lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return str(s.getsockname()[0])
    finally:
        s.close()

print("My LAN IP is:", get_lan_ip())

HEADER = 64  
PORT = 5050
SERVER = get_lan_ip() 
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"

shutdown = False

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
# Disable Nagle's algorithm by setting TCP_NODELAY to 1
server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
server.bind(ADDR)

def handle_client(conn, addr): 
    global shutdown
    print(f"[NEW CONNECTION] {addr} connected.")

    connected = True
    while connected:
        msg_length = conn.recv(HEADER)
        if msg_length:
            msg_length = int(msg_length)
            # actual message 
            msg = conn.recv(msg_length)
            print(msg)
            message = pickle.loads(msg)
            print(message)
            if message == DISCONNECT_MESSAGE:
                connected = False
            print(f"[{addr}] {message} {type(message)}")
            conn.send("Msg received".encode(FORMAT))

    conn.close()
    print("Connection closed")

def start():
    server.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")
    while True:
        conn, addr = server.accept() 
        thread = threading.Thread(target=handle_client, args=(conn, addr))
        thread.start()

        print(f"[ACTIVE CONNECTIONS] {threading.active_count() - 1}") 

print("[STARTING] server is starting...")
start()
