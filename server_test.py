import network
import socket
import time

from machine import Pin

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

