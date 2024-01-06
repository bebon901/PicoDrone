# importing pygame module
import pygame
import requests
# importing sys module
import sys
 
# initialising pygame
pygame.init()
pico_address = "http://192.168.137.200:80"
# creating display
display = pygame.display.set_mode((300, 300))
 
# creating a running loop
while True:
       
    # creating a loop to check events that
    # are occurring
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
         
        # checking if keydown event happened or not
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                r = requests.get(url = pico_address + "/down/w")
            if event.key == pygame.K_a:
                r = requests.get(url = pico_address + "/down/a")
            if event.key == pygame.K_s:
                r = requests.get(url = pico_address + "/down/s")
            if event.key == pygame.K_d:
                r = requests.get(url = pico_address + "/down/d")
            if event.key == pygame.K_UP:
                r = requests.get(url = pico_address + "/down/kup")
            if event.key == pygame.K_DOWN:
                r = requests.get(url = pico_address + "/down/kdown")
            if event.key == pygame.K_0:
                r = requests.get(url = pico_address + "/kill")
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_w:
                r = requests.get(url = pico_address + "/up/w")
            if event.key == pygame.K_a:
                r = requests.get(url = pico_address + "/up/a")
            if event.key == pygame.K_s:
                r = requests.get(url = pico_address + "/up/s")
            if event.key == pygame.K_d:
                r = requests.get(url = pico_address + "/up/d")
            if event.key == pygame.K_UP:
                r = requests.get(url = pico_address + "/up/kup")
            if event.key == pygame.K_DOWN:
                r = requests.get(url = pico_address + "/up/kdown")
