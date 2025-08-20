import numpy as np
import serial
import time

PORT = '/dev/ttyACM0'
BAUDRATE = 921600

ser = serial.Serial(
    port = PORT,
    baudrate = BAUDRATE,
    timeout = 1
)

time.sleep(2)

array = np.random.rand(10000)

# format message as <X:X:X> 
#msg = f"<{':'.join(map(str, array))}>" 

msg = "<0:1.0:2.1:3.2:n:1.9:5.1:5.2:n:2.0:3.1:4.2:n:9.9:7.1:3.5:n:1.8:4.4:3.3:l>"

ser.write(msg.encode())
#print("Sent msg: ", msg)



