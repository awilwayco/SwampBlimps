#!/usr/bin/env python3

"""
pip3 install pygame
pip3 install pySerial
pip3 install easygui
sudo apt-get install python3-tk (to install tkinter, needed for easygui)
"""
import pygame
from threading import Thread
import time
#Check if valid Wi-Fi
import subprocess

from BlimpHandler import BlimpHandler
from Display import Display

def main():
    global alive
    alive = True
    pygame.init()

    blimpHandler = BlimpHandler()
    display = Display(blimpHandler)

    #LOOP
    thread_blimpHandler = Thread(target=asyncBlimpHandler,args={blimpHandler})
    thread_display = Thread(target=asyncDisplayDraw,args={display})

    thread_blimpHandler.start()
    thread_display.start()
    while alive:
        time.sleep(0.001)
        display.updateEvent()
        alive = display.alive
        if(alive==False): print("Display killed.")

    thread_blimpHandler.join()
    thread_display.join()
    blimpHandler.close()
    display.close()

def asyncBlimpHandler(blimpHandler):
    while alive:
        time.sleep(0.001)
        blimpHandler.update()

def asyncDisplayDraw(display):
    while alive:
        time.sleep(0.001)
        display.updateDraw()

def check_wifi_ssid():
    output = subprocess.check_output(["iwconfig", "2>/dev/null | grep 'ESSID:' | cut -d '\"' -f 2"], shell=True)
    ssid = output.decode('utf-8').strip()
    if(ssid.find("COREBlimp") == -1 and ssid.find("COREBlimp_5G_1") == -1):
        print("Invalid WiFi selected")
        return False
    else:
        return True

if __name__ == '__main__':
    if(check_wifi_ssid()):
        main()