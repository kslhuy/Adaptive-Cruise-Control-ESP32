from serial.tools import list_ports
import serial
import time
import csv
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

# Identify the correct port
ports = list_ports.comports()
for port in ports: print(port)


# Initialize lists to hold the data points for plotting
time_data = []
value1_data = []
value2_data = []
value3_data = []

# Create CSV file
f = open("data.csv","w",newline='')
f.truncate()

# Open the serial com
serialCom = serial.Serial('COM6',115200)

# # Toggle DTR to reset the Arduino
# serialCom.setDTR(False)
# time.sleep(1)
# serialCom.flushInput()
# serialCom.setDTR(True)

# How many data points to record
kmax = 20*90
initial_display_range = 20
# Create a figure and axis for the plot
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='Value 1')
line2, = ax.plot([], [], label='Value 2')
line3, = ax.plot([], [], label='Value 3')

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    ax.set_xlim(0, initial_display_range)
    ax.set_ylim(-5, 5)  # Adjust the y-limit as needed
    ax.set_xlabel("Time")
    ax.set_ylabel("Value")
    ax.set_title("Real-time Data Plot")
    ax.legend()
    return line1, line2, line3

# Loop through and collect data as it is available
def update(frame):
    try:
        # Read the line
        s_bytes = serialCom.readline()
        decoded_bytes = s_bytes.decode("utf-8").strip('\r\n')
        # print(decoded_bytes)


        values = [float(x) for x in decoded_bytes.split()]
        print(values)

        time_data.append(values[0])
        value1_data.append(values[1])
        value2_data.append(values[2])
        # value3_data.append(values[3])

        line1.set_data(time_data, value1_data)
        line2.set_data(time_data, value2_data)
        # line3.set_data(time_data, value3_data)
    except:
        print("Error encountered, line was not recorded.")

    return line1, line2, line3
        # Initialize a list to hold the data points for plotting
ani = FuncAnimation(fig, update, frames=range(kmax),
                    init_func=init, blit=True)


try:
    plt.show()
except KeyboardInterrupt:
    print("\nCtrl+C detected. Exiting...")
finally:
    if serialCom.is_open:
        serialCom.close()
    f.close()
