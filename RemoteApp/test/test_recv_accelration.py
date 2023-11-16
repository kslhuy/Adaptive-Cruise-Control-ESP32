# import socket

# HOST = '192.168.85.251'  # Listen on all available interfaces
# PORT = 9997      # Choose a suitable port number

# server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server_socket.bind((HOST, PORT))
# server_socket.listen(1)

# print(f"Server listening on {HOST}:{PORT}")

# client_socket, client_address = server_socket.accept()
# print(f"Connected by {client_address}")

# while True:
#     data = client_socket.recv(1024)  # Adjust buffer size as needed
#     if not data:
#         break
#     data_sets = data.decode().strip().split('\n')
#     # data_type, data_values = data_line.split(':', 1)  # Split data type and values

    
#     for data_line in data_sets:
#         data_line = data_line.split(':', 1)  # Split data type and values

#         if (len(data_line) > 1):
#             data_type = data_line[0]
#             data_values = data_line[1]
#             if data_type == "GYO":
#                 # rotation_values = [float(val) for val in data_values.strip().split(',')]
#                 print(f"Received gyroscope data from client : {data_values}")
            
#             elif data_type == "ACC":
#                 # acc_values = [float(val) for val in data_values.strip().split(',')]
#                 print(f"Received acceleration data from client : {data_values}")

#         # if len(data_values) == 3:
#         #     try:
#         #         x = float(data_values[0].strip())
#         #         y = float(data_values[1].strip())
#         #         z = float(data_values[2].strip())
#         #         print(f"Received data: x={x}, y={y}, z={z}")
#         #         # Process the data as needed
#         #     except ValueError:
#         #         print(f"Invalid data: {data_line}")
#         # else:
#         #     print(f"Incomplete data: {data_line}")

# import socket

# # Define the server address and port
# server_address = ('192.168.85.251', 5555)

# # Create a UDP socket
# udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# # Bind the socket to the server address and port
# udp_socket.bind(server_address)

# print("Server is listening...")

# while True:
#     try:
#         # Receive data from the client
#         data, client_address = udp_socket.recvfrom(1024)
        
#         if not data:
#             break
#         # Print the received data and client address
#         print(f" {data.decode('utf-8')}")
        
#         # # Optionally, send a response back to the client
#         # response = "Received your data!"
#         # udp_socket.sendto(response.encode('utf-8'), client_address)
        
#     except KeyboardInterrupt:
#         print("Server stopped.")
#         break

# # Close the UDP socket
# udp_socket.close()




import socket
import signal
import sys
import time

# Function to parse linear acceleration data
def parse_linear_acceleration(data):
    linear_accel_data = []
    # print(data , len(data))
    # Iterate through the data in groups of 4 values
    for i in range(0, len(data)):
        try:
            sensor_type = int(data[i])
            # print(sensor_type)

            if sensor_type == 82:  # Sensor type 5 corresponds to Linear Acceleration Data
                # print(sensor_type)
                linear_accel_data.extend(map(float, data[i + 1:i + 4]))
        except ValueError:
            pass  # Skip non-integer sensor types

    return linear_accel_data


# Set up the UDP socket
server_address = ('192.168.85.251', 5555)

BUFFER_SIZE = 1024

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(server_address)
# sock.listen(1)
print("Listening for data...")

idle_timeout = 5  # Timeout in seconds
last_data_time = time.time()

def shutdown(signal, frame):
    print("Shutting down...")
    sock.close()
    sys.exit(0)

# Register the signal handler for Ctrl+C
signal.signal(signal.SIGINT, shutdown)

while True:
    data = sock.recv(BUFFER_SIZE)
    if not data:
        print("No data received. Shutting down...")
        sock.close()
        sys.exit(0)

    last_data_time = time.time()

    data = data.decode("utf-8").split(',')
    linear_accel_data = parse_linear_acceleration(data)
    print("Received Linear Acceleration Data:", linear_accel_data)
    # print("Received", data.decode('utf-8'))

    # Check for idle timeout
    if time.time() - last_data_time > idle_timeout:
        print("Idle timeout reached. Shutting down...")
        sock.shutdown()
        # sock.close()
        break
    # else :
    #     print("not...")
