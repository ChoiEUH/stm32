import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("0.0.0.0", 5000))
s.listen(1)
print("Listening on 0.0.0.0:5000")
conn, addr = s.accept()
print("Connected by", addr)
while True:
    data = conn.recv(1024)
    if not data: break
    print("RX:", data.decode(errors="ignore"))
