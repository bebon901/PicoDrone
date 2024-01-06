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
import MPU6050

# Set up the I2C interface
i2c = machine.I2C(0, sda=machine.Pin(16), scl=machine.Pin(17))

# Set up the MPU6050 class 
mpu = MPU6050.MPU6050(i2c)

# wake up the MPU6050 from sleep
mpu.wake()

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
pid_xp = 0.5
pid_yd = 0
pid_yi = 0
pid_yp = 0.5

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
    #status = wlan.ifconfig()
    #print(status[0])



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
    err_gyro_z = 0
    ang_x = ang_y = ang_gyr_x = ang_gyr_y = 0
    print("Calibrating Gyro, please remain still")
    for i in range(200):
        x, y, z = mpu.read_gyro_data()
        err_gyro_x += x
        err_gyro_y += y
        err_gyro_z += z
    err_gyro_x = err_gyro_x / 200
    err_gyro_y = err_gyro_y / 200
    err_gyro_z = err_gyro_z / 200
    print("All done!! Starting PID now")
    previous_time = time.ticks_us()
    while run_pid:
        ang_xs = []
        ang_ys = []
        for i in range(20):
            gyr_x, gyr_y, gyr_z = mpu.read_gyro_data()
            acc_x, acc_y, acc_z = mpu.read_accel_data()
            new_time = time.ticks_us()
            gyr_x -= err_gyro_x
            gyr_y -= err_gyro_y
            gyr_z -= err_gyro_z
            angc_gyr_x = gyr_x*(new_time - previous_time)/1000000 # microseconds to seconds
            angc_gyr_y = gyr_y*(new_time - previous_time)/1000000 # microseconds to seconds
            previous_time = new_time
            
            ang_acc_y = math.atan(acc_x / (acc_y**2 + acc_z**2)**0.5) * 360 / math.pi / 2
            ang_acc_x = math.atan(acc_y / (acc_x**2 + acc_z**2)**0.5) * 360 / math.pi / 2
            
            # Combine the two measurements...
            ang_x = 0.5 * (ang_x + angc_gyr_x) + 0.5 * ang_acc_x
            ang_y = 0.5 * (ang_y + angc_gyr_y) + 0.5* ang_acc_y
            
            ang_xs.append(ang_x)
            ang_ys.append(ang_y)
        ang_x = 0
        ang_y = 0
        for r in range(len(ang_xs)):
            ang_x += ang_xs[r] / len(ang_xs)
            ang_y += ang_ys[r] / len(ang_xs)
        ## DIRECTIONS ARE ORTHOGONAL to the drone, will change that here
        use_ang_x = ang_y
        use_ang_y = ang_x
        
        prev_err_x = err_x
        prev_err_y = err_y
        
        err_x = use_ang_x - des_x
        err_y = use_ang_y - des_y
        
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
        '''
        # Print results
        print("X:", "{:.2f}".format(ang_x), \
              "| Y:", "{:.2f}".format(ang_y), \
              "| Z:", "{:.2f}".format(acc_z))
        
        # Print results
        print("FL:", "{:.2f}".format(val_fl), \
              "| FR:", "{:.2f}".format(val_fr), \
              "| BL:", "{:.2f}".format(val_bl), \
              "| BR:", "{:.2f}".format(val_br))
        print("FL:", "{:.2f}".format(int(val_fl / 100 * 65535)), \
              "| FR:", "{:.2f}".format(int(val_fr / 100 * 65535)), \
              "| BL:", "{:.2f}".format(int(val_bl / 100 * 65535)), \
              "| BR:", "{:.2f}".format(int(val_br / 100 * 65535)))
        '''
        print(ang_x, ang_y, angc_gyr_x, angc_gyr_y, err_gyro_y)
        #utime.sleep(0.01)    
            
            
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
        
        if request.find('/kill') != -1:
            throttle = 0
        
        if request.find('/kp/') != -1:
            print("KPING!!!")
            kp = float(request[request.find("/kp/")+4:request.find("/valuekp")])
            print(kp)
            pid_xp = pid_yp = kp

        response = html % "Hi"

        cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
        cl.send(response)
        cl.close()

    except OSError as e:
        cl.close()
        print('connection closed')
    except KeyboardInterrupt:
        #cl.close()
        run_pid = False
        break
run_pid = False

