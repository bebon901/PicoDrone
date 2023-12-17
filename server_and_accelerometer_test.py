import network
import socket
import time
import _thread
from machine import Pin
import machine
import utime
import ustruct
import sys

led = Pin(15, Pin.OUT)

ssid = 'Bens Laptop'
password = 'password'

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

html = """<!DOCTYPE html>
<html>
    <head> <title>Pico W</title> </head>
    <body> <h1>Pico W</h1>
        <p>%s</p>
    </body>
</html>
"""

max_wait = 10
while max_wait > 0:
    if wlan.status() < 0 or wlan.status() >= 3:
        break
    max_wait -= 1
    print('waiting for connection...')
    time.sleep(1)

if wlan.status() != 3:
    raise RuntimeError('network connection failed')
else:
    print('connected')
    status = wlan.ifconfig(('192.168.137.200', '255.255.255.0', '192.168.1.1', '8.8.8.8'))
    print( 'ip = 192.168.137.200')

addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]

s = socket.socket()
s.bind(addr)
s.listen(1)

print('listening on', addr)


###############################################################################
# Constants

# Registers
REG_DEVID = 0x00
REG_POWER_CTL = 0x2D
REG_DATAX0 = 0x32

# Other constants
DEVID = 0xE5
SENSITIVITY_2G = 1.0 / 256  # (g/LSB)
EARTH_GRAVITY = 9.80665     # Earth's gravity in [m/s^2]

###############################################################################
# Settings

# Assign chip select (CS) pin (and start it high)
cs = machine.Pin(17, machine.Pin.OUT)

# Initialize SPI
spi = machine.SPI(0,
                  baudrate=1000000,
                  polarity=1,
                  phase=1,
                  bits=8,
                  firstbit=machine.SPI.MSB,
                  sck=machine.Pin(18),
                  mosi=machine.Pin(19),
                  miso=machine.Pin(16))

###############################################################################
# Functions

def reg_write(spi, cs, reg, data):
    """
    Write 1 byte to the specified register.
    """
    
    # Construct message (set ~W bit low, MB bit low)
    msg = bytearray()
    msg.append(0x00 | reg)
    msg.append(data)
    
    # Send out SPI message
    cs.value(0)
    spi.write(msg)
    cs.value(1)
    
def reg_read(spi, cs, reg, nbytes=1):
    """
    Read byte(s) from specified register. If nbytes > 1, read from consecutive
    registers.
    """
    
    # Determine if multiple byte (MB) bit should be set
    if nbytes < 1:
        return bytearray()
    elif nbytes == 1:
        mb = 0
    else:
        mb = 1
    
    # Construct message (set ~W bit high)
    msg = bytearray()
    msg.append(0x80 | (mb << 6) | reg)
    
    # Send out SPI message and read
    cs.value(0)
    spi.write(msg)
    data = spi.read(nbytes)
    cs.value(1)
    
    return data

###############################################################################
# Main

# Start CS pin high
cs.value(1)

# Workaround: perform throw-away read to make SCK idle high
reg_read(spi, cs, REG_DEVID)

# Read device ID to make sure that we can communicate with the ADXL343
data = reg_read(spi, cs, REG_DEVID)
if (data != bytearray((DEVID,))):
    print("ERROR: Could not communicate with ADXL343")
    sys.exit()
    
# Read Power Control register
data = reg_read(spi, cs, REG_POWER_CTL)
print(data)

reg_write(spi, cs, 0x2C, 12)
# Tell ADXL343 to start taking measurements by setting Measure bit to high
data = int.from_bytes(data, "big") | (1 << 3)
reg_write(spi, cs, REG_POWER_CTL, data)

# Test: read Power Control register back to make sure Measure bit was set
data = reg_read(spi, cs, REG_POWER_CTL)
print(data)

# Wait before taking measurements
utime.sleep(2.0)


def server_remote():
    # Listen for connections
    while True:
        try:
            cl, addr = s.accept()
            print('client connected from', addr)
            request = cl.recv(1024)
            request = str(request)
            #print(request)
            #print(request.find('/up/down'))
            if request.find('/up/w') != -1:
                print("up/w")
            if request.find('/up/a') != -1:
                print("up/a")
            if request.find('/up/s') != -1:
                print("up/s")
            if request.find('/up/d') != -1:
                print("up/d")
            if request.find('/up/kup') != -1:
                print("up/up")
            if request.find('/up/kdown') != -1:
                print("up/down")
                
            
            if request.find('/down/w') != -1:
                print("down/w")
            if request.find('/down/a') != -1:
                print("down/a")
            if request.find('/down/s') != -1:
                print("down/s")
            if request.find('/down/d') != -1:
                print("down/d")
            if request.find('/down/kup') != -1:
                print("down/up")
            if request.find('/down/kdown') != -1:
                print("down/down")

            response = html % "Hi"

            cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
            cl.send(response)
            cl.close()

        except OSError as e:
            cl.close()
            print('connection closed')
            
            
## START WEBPAGE SERVER
_thread.start_new_thread(server_remote, ())

## RUN PID CONTROLLER ETC!
while True:
    
    # Read X, Y, and Z values from registers (16 bits each)
    data = reg_read(spi, cs, REG_DATAX0, 6)

    # Convert 2 bytes (little-endian) into 16-bit integer (signed)
    acc_x = ustruct.unpack_from("<h", data, 0)[0]
    acc_y = ustruct.unpack_from("<h", data, 2)[0]
    acc_z = ustruct.unpack_from("<h", data, 4)[0]

    # Convert measurements to [m/s^2]
    acc_x = acc_x * SENSITIVITY_2G * EARTH_GRAVITY
    acc_y = acc_y * SENSITIVITY_2G * EARTH_GRAVITY
    acc_z = acc_z * SENSITIVITY_2G * EARTH_GRAVITY

    # Print results
    print("X:", "{:.2f}".format(acc_x), \
          "| Y:", "{:.2f}".format(acc_y), \
          "| Z:", "{:.2f}".format(acc_z))
    
    #utime.sleep(0.1)
