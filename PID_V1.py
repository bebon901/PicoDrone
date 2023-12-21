import network
import socket
import time
import _thread
from machine import Pin, PWM
import machine
import utime
import ustruct
import sys
import math

led = Pin(15, Pin.OUT)

fl = machine.Pin(22)
fr = machine.Pin(26)
bl = machine.Pin(28)
br = machine.Pin(21)

mot_fl = PWM(fl)
mot_fr = PWM(fr)
mot_bl = PWM(bl)
mot_br = PWM(br)

frequency = 5000
mot_fl.freq (frequency)
mot_fr.freq (frequency)
mot_bl.freq (frequency)
mot_br.freq (frequency)

## Set te desired angles variables
des_x = 0
des_y = 0

## Set the PID Constants
pid_xd = 0
pid_xi = 0
pid_xp = 1

pid_yd = 0
pid_yi = 0
pid_yp = 1


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
    print('ip = 192.168.137.200')



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

reg_write(spi, cs, 0x2C, 15)
# Tell ADXL343 to start taking measurements by setting Measure bit to high
data = int.from_bytes(data, "big") | (1 << 3)
reg_write(spi, cs, REG_POWER_CTL, data)

# Test: read Power Control register back to make sure Measure bit was set
data = reg_read(spi, cs, REG_POWER_CTL)
print(data)

# Wait before taking measurements
utime.sleep(2.0)

## Set te desired angles variables
des_x = 0
des_y = 0

run_pid = True
throttle = 0 # 0-100 pls
def cap_val(val, lower, upper):
    if val < lower:
        val = lower
    if val > upper:
        val = upper
    return val
def median(data):
    data = sorted(data)
    n = len(data)
    if n % 2 == 1:
        return data[n//2]
    else:
        i = n//2
        return (data[i - 1] + data[i])/2
def read_and_print_accelerometer():
    global throttle, des_x, des_y, pid_xp, pid_xi, pid_xd, pid_yp, pid_yi, pid_yd, mot_fr, mot_fl, mot_bl, mot_br, run_pid
    err_x = err_y = x_pid_i = y_pid_i = 0
    ## Calibrate the gyro:
    err_gyro_x = 0
    err_gyro_y = 0
    for i in range(200):
        err_gyro_x += 
    while run_pid:
        ang_x = []
        ang_y = []
        for i in range(200):
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
            
            # Compute angles
            try:
                ang_y.append(math.atan(acc_x / (acc_y**2 + acc_z**2)**0.5) * 360 / math.pi / 2)
                ang_x.append(math.atan(acc_y / (acc_x**2 + acc_z**2)**0.5) * 360 / math.pi / 2)
            except ZeroDivisionError:
                print("DIVIDE BY ZERO FAILED!!")
        print(ang_x)
        ang_x = median(ang_x)
        ang_y = median(ang_y)
        
        
        prev_err_x = err_x
        prev_err_y = err_y
        
        err_x = ang_x - des_x
        err_y = ang_y - des_y
        
        x_pid_p = err_x * pid_xp
        y_pid_p = err_y * pid_yp
        
        x_pid_i += err_x * pid_xi
        y_pid_i += err_y * pid_yi

        x_pid_d = (err_x - prev_err_x) * pid_xd
        y_pid_d = (err_y - prev_err_y) * pid_yd
        
        val_x = x_pid_p + x_pid_i + x_pid_d
        val_y = y_pid_p + y_pid_i + y_pid_d
        
        ### MOTOR OUTPUT VALUES ###
        val_fl = cap_val(throttle + val_x + val_y, 0, 100)
        val_fr = cap_val(throttle + val_x - val_y, 0, 100)
        val_bl = cap_val(throttle - val_x + val_y, 0, 100)
        val_br = cap_val(throttle - val_x - val_y, 0, 100)
        
        mot_fl.duty_u16(int(val_fl / 100 * 65535))        
        mot_fr.duty_u16(int(val_fr / 100 * 65535))        
        mot_bl.duty_u16(int(val_bl / 100 * 65535))       
        mot_br.duty_u16(int(val_br / 100 * 65535))
        
        # Print results
        '''print("X:", "{:.2f}".format(ang_x), \
              "| Y:", "{:.2f}".format(ang_y), \
              "| Z:", "{:.2f}".format(acc_z))'''
        # Print results
        print("FL:", "{:.2f}".format(val_fl), \
              "| FR:", "{:.2f}".format(val_fr), \
              "| BL:", "{:.2f}".format(val_bl), \
              "| BR:", "{:.2f}".format(val_br))
        print("FL:", "{:.2f}".format(int(val_fl / 100 * 65535)), \
              "| FR:", "{:.2f}".format(int(val_fr / 100 * 65535)), \
              "| BL:", "{:.2f}".format(int(val_bl / 100 * 65535)), \
              "| BR:", "{:.2f}".format(int(val_br / 100 * 65535)))
        
        #utime.sleep(0.1)    
            
            
## START WEBPAGE SERVER

_thread.start_new_thread(read_and_print_accelerometer, ())

addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]

s = socket.socket()
s.bind(addr)
s.listen(1)

print('listening on', addr)
# Listen for connections
while True:
    try:
        cl, addr = s.accept()
        print('client connected from', addr)
        request = cl.recv(1024)
        request = str(request)
        print(request)
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
            throttle += 10
        if request.find('/down/kdown') != -1:
            print("down/down")
            throttle -= 10

        response = html % "Hi"

        cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
        cl.send(response)
        cl.close()

    except OSError as e:
        cl.close()
        print('connection closed')
    except KeyboardInterrupt:
        cl.close()
        run_pid = False
run_pid = False
