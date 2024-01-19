import socket
import time 

# PC IP address and port number
pc_ip = "192.168.137.1"  # Replace with your PC's IP address
pc_port = 1234  # Replace with your chosen port number

print (socket.gethostname())
# Create a TCP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the PC's IP address and port number
server_socket.bind((pc_ip, pc_port))

# Listen for incoming connections
server_socket.listen(5)

print("PC server started. Waiting for connections...")

# Accept a client connection
client_socket, client_address = server_socket.accept()
print("ESP32 connected:", client_address)

# Receive data from the ESP32
data = client_socket.recv(1024).decode()
print("Received from ESP32:", data)


time.sleep(3)

# Send response back to the ESP32
response = "Hello, ESP32!, its PC\n"
# client_socket.send(response.encode())

for i in range(1,5):
    time.sleep(2);    
    client_socket.send(response.encode())

client_socket.close()
# Close the connection
# client_socket.close()
# server_socket.close()
