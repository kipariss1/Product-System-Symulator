import time
import threading
import sys
import copy
import pygame
import math
from matplotlib import pyplot as plt

try:
    import smbus
    import RPi.GPIO as GPIO
    DEBUG = False
except:                                     # execute the try part only if exception is true
    DEBUG = True

print('Program System Symulator  Copyright (C) 2020  Kirill Rassudikhin \n
    This program comes with ABSOLUTELY NO WARRANTY \n
    This is free software, and you are welcome to redistribute it \n
    under certain conditions;')


# RPi implementation HERE:
def simulation_part(lock, payload):
    with lock:
        t = tc                              # coping the time of the cycle into the function

    # setup GPIO
    GPIO.setmode(GPIO.BCM)
    channel = 4                             # this is channel I01 on the UniPi
    GPIO.setup(channel, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # pull up the pin (simulation of Y1)

    # setup I2C relays
    bus = smbus.SMBus(1)

    bus.write_byte_data(0x20, 0x00, 0x00)   # configuring the relays in right mode
    bus.write_byte_data(0x20, 0x09, 128)    # setup start position

    # setup real time counter
    stime = time.time()                     # real time counter
    ref = time.time()                       # reference during the loop
    a = time.time()
    b = time.time()
    while (b - a) < r:                      # simulating of the delay due to reaction time of the sensors
        print('start position has been set')
        b = time.time()
        with lock:                          # here writing the metadata about the simulation for visualisation
            payload["switch1"] = True
            payload["switch2"] = False
            payload["direction"] = 1

    # main loop
    while alive:
        # print(ref - stime)     # for testing of time of the cycle
        if not GPIO.input(channel) and (stime - ref < (t-r/2)):     # both relays are off, transition state, moving to
                                                                    # end state
            bus.write_byte_data(0x20, 0x09, 0)                      # turn off relays
            stime = time.time()
            with lock:                                              # metadata about the sim for viz
                payload["switch1"] = False
                payload["switch2"] = False
                payload["direction"] = 1
        elif not GPIO.input(channel) and (stime - ref >= t):        # second relay is on, end state
            bus.write_byte_data(0x20, 0x09, 64)                     # turn on the 2 relay (simulation of 1M2)
            a = time.time()
            b = time.time()
            while (b - a) < r:                                      # simulating the delay due to reaction time of the
                                                                    # sensors
                print('end position has been set')
                b = time.time()
                with lock:                                          # metadata about the sim for viz
                    payload["switch1"] = False
                    payload["switch2"] = True
                    payload["direction"] = -1
            stime = time.time()
        if GPIO.input(channel) and (stime - ref < 2*t-r):             # both relays are off, transition state, moving to
                                                                    # start state
            bus.write_byte_data(0x20, 0x09, 0)                      # turn off relays
            stime = time.time()
            with lock:  # metadata about the sim for viz
                payload["switch1"] = False
                payload["switch2"] = False
                payload["direction"] = -1
        elif GPIO.input(channel) and (stime - ref >= 2*t):          # first relay is on , start state
            bus.write_byte_data(0x20, 0x09, 128)                    # turn on the 1 relay (simulation of 1M1)
            a = time.time()
            b = time.time()
            while (b - a) < r:                                      # simulating the delay due to reaction time of the
                                                                    # sensors
                print('start! position has been set')
                b = time.time()
                with lock:                                          # writing the metadata about the simulation for
                                                                    # visualisation
                    payload["switch1"] = True
                    payload["switch2"] = False
                    payload["direction"] = -1
            stime = time.time()
            ref = time.time()


# RPi dummy implementation for testing on PC here:
def dummy_simulation_part(lock, payload):
    ts = 0.25               # time sleep
    t = tc                  # coping the time of the cycle into the dummy function
    GPIO = False            # Dummy simulation of GPIO Input
    stime = time.time()     # real time counter
    ref = time.time()       # reference during the loop
    a = time.time()
    b = time.time()
    while alive and (b - a) < r:      # simulating of the delay due to reaction time of the sensors
        print('start position has been set')
        b = time.time()
        with lock:          # metadata about the sim for viz
            payload["switch1"] = True
            payload["switch2"] = False
            payload["direction"] = 1
        time.sleep(ts)

    # main loop
    while alive:
        print(ref - stime)     # for testing of time of the cycle
        print(payload)
        if not GPIO and (stime - ref < (t-r)):  # both relays are off, transition state, moving to end state
            stime = time.time()
            with lock:  # metadata about the sim for viz
                payload["switch1"] = False
                payload["switch2"] = False
                payload["direction"] = 1
            time.sleep(ts)
        elif not GPIO and (stime - ref >= (t-r)):  # second relay is on, end state
            a = time.time()
            b = time.time()
            while (b - a) < r:  # simulating the delay due to reaction time of the seniors
                print('end position has been set')
                b = time.time()
                with lock:  # metadata about the sim for viz
                    payload["switch1"] = False
                    payload["switch2"] = True
                    payload["direction"] = -1
                print(payload)
                time.sleep(ts)
            stime = time.time()
            GPIO = True
        if GPIO and (stime - ref < 2 * t):  # both relays are off, transition state, moving to start state
            stime = time.time()
            with lock:  # metadata about the sim for viz
                payload["switch1"] = False
                payload["switch2"] = False
                payload["direction"] = -1
            time.sleep(ts)
        elif GPIO and (stime - ref >= 2 * t):  # first relay is on , start state
            a = time.time()
            b = time.time()
            while (b - a) < r:  # simulating the delay due to reaction time of the sensors
                print('start! position has been set')
                b = time.time()
                with lock:  # metadata about the sim for viz
                    payload["switch1"] = True
                    payload["switch2"] = False
                    payload["direction"] = -1
                print(payload)
                time.sleep(ts)
            stime = time.time()
            ref = time.time()  # zeroing the reference
            GPIO = False


# FUNCTION, WHICH VISUALIZES SIMULATION HERE:
def vizualization_part(lock, payload):
    FPS = 24

    pygame.init()  # initialize pygame

    win = pygame.display.set_mode((600, 400))  # define the window with visualisation
    pygame.display.set_caption("Visualisation of the pneumatic cylinder")  # define the name of the window
    clock = pygame.time.Clock()  # setup for method "clock" to limit FPS

    # system of coordinates

    # the cylinder:
    x = 250
    y = 150
    # those x,y coordinates are coordinates of the left upper point of the cylinder
    # left wall:    (coordinates, height and width)
    hc1 = 150
    wc1 = 10
    xc1 = x
    yc1 = y

    # floor:    (coordinates, height and width)
    hc3 = 10
    wc3 = 40
    xc3 = x + wc1
    yc3 = y + hc1 - hc3

    # right wall:   (coordinates, height and width)
    hc2 = hc1
    wc2 = wc1
    xc2 = xc3 + wc3
    yc2 = y

    # ceiling:  (coordinates, height and width)
    hc4 = hc3
    wc4 = wc3
    xc4 = x + wc1
    yc4 = y

    # the piston:   (coordinates, height and width)
    x1 = math.ceil(x + wc1 + wc3 / 2)
    y1 = y - 70

    # circle:   (coordinates, radius)
    xc = x1
    yc = y1
    rc = 15

    # connecting rod:   (coordinates, height and width)
    woi = 10
    xoi = math.ceil(xc - woi / 2)
    yoi = yc + rc
    hoi = math.ceil(hc1 / 1.5)

    # head of the piston:   (coordinates, height and width)
    wp = wc3
    hp = 10
    xp = xc - wp / 2
    yp = yoi + hoi

    # sensors:  (coordinates, height and width)
    ws1 = 20
    hs1 = ws1
    xs1 = xc + 100
    ys1 = yc - hs1
    ws2 = 20
    hs2 = ws2
    xs2 = xc + 100
    ys2 = yc + 40

    # down arrow:   (coordinates of three points of the polygon)
    xda1 = 50
    yda1 = 150
    xda2 = 150
    yda2 = yda1
    xda3 = 100
    yda3 = yda1 + 50

    # up arrow:     (coordinates of three points of the polygon)
    xua1 = 50
    yua1 = 200
    xua2 = 150
    yua2 = yua1
    xua3 = 100
    yua3 = yua1 - 50

    while alive:
        with lock:
            data = copy.copy(payload)       # getting the metadata from simulation function
        for event in pygame.event.get():    # check every event and set the counter to True if it is the click on X
                                            # button
            if event.type == pygame.QUIT:   # if you press "X" - pygame will close the window
                pygame.quit()
                sys.exit()
        # drawing of the static objects here:
        pygame.draw.rect(win, (0, 0, 255), (xc1, yc1, wc1, hc1))  # cylinder
        pygame.draw.rect(win, (0, 0, 255), (xc2, yc2, wc2, hc2))  # cylinder
        pygame.draw.rect(win, (0, 0, 255), (xc3, yc3, wc3, hc3))  # cylinder
        pygame.draw.rect(win, (0, 0, 255), (xc4, yc4, wc4, hc4))  # cylinder
        pygame.draw.rect(win, (255, 0, 0), (xs1, ys1, ws1, hs1))  # sensors
        pygame.draw.rect(win, (255, 0, 0), (xs2, ys2, ws2, hs2))  # sensors
        # drawing of moving objects here:
        if data["switch1"] and (data["direction"] == 1):
            pygame.draw.rect(win, (0, 255, 0), (xs1, ys1, ws1, hs1))    # sensor
            pygame.draw.circle(win, (0, 0, 255), (xc, yc), rc)          # head of the piston
            pygame.draw.rect(win, (0, 0, 255), (xoi, yoi, woi, hoi))    # connecting rod
            pygame.draw.rect(win, (0, 0, 255), (xp, yp, wp, hp))        # piston
            pygame.draw.polygon(win, (0, 255, 0), [[xda1, yda1], [xda2, yda2], [xda3, yda3]])   # arrow
        if data["switch1"] and (data["direction"] == -1):
            pygame.draw.rect(win, (0, 255, 0), (xs1, ys1, ws1, hs1))    # sensor
            pygame.draw.circle(win, (0, 0, 255), (xc, yc), rc)          # head of the piston
            pygame.draw.rect(win, (0, 0, 255), (xoi, yoi, woi, hoi))    # connecting rod
            pygame.draw.rect(win, (0, 0, 255), (xp, yp, wp, hp))        # piston
            pygame.draw.polygon(win, (0, 255, 0), [[xua1, yua1], [xua2, yua2], [xua3, yua3]])   # arrow
        if not data["switch1"] and not data["switch2"]:
            if data["direction"] == 1:
                pygame.draw.circle(win, (0, 0, 255), (xc, yc), rc)      # head of the piston
                pygame.draw.rect(win, (0, 0, 255), (xoi, yoi, woi, hoi))    # connecting road
                pygame.draw.rect(win, (0, 0, 255), (xp, yp, wp, hp))    # piston
                pygame.draw.polygon(win, (0, 255, 0), [[xda1, yda1], [xda2, yda2], [xda3, yda3]])   # arrow
            else:
                pygame.draw.circle(win, (0, 0, 255), (xc, ys2), rc)     # head of the piston
                pygame.draw.rect(win, (0, 0, 255), (xoi, yoi - (yc - ys2), woi, hoi))   # connecting rod
                pygame.draw.rect(win, (0, 0, 255), (xp, yp - (yc - ys2), wp, hp))   # piston
                pygame.draw.polygon(win, (0, 255, 0), [[xua1, yua1], [xua2, yua2], [xua3, yua3]])   # arrow
        if data["switch2"] and (data["direction"] == 1):
            pygame.draw.rect(win, (0, 255, 0), (xs2, ys2, ws2, hs2))    # sensor
            pygame.draw.circle(win, (0, 0, 255), (xc, ys2), rc)         # head of the piston
            pygame.draw.rect(win, (0, 0, 255), (xoi, yoi - (yc - ys2), woi, hoi))   # connecting rod
            pygame.draw.rect(win, (0, 0, 255), (xp, yp - (yc - ys2), wp, hp))       # piston
            pygame.draw.polygon(win, (0, 255, 0), [[xda1, yda1], [xda2, yda2], [xda3, yda3]])   # arrow
        if data["switch2"] and (data["direction"] == -1):
            pygame.draw.rect(win, (0, 255, 0), (xs2, ys2, ws2, hs2))    # sensor
            pygame.draw.circle(win, (0, 0, 255), (xc, ys2), rc)         # head of the piston
            pygame.draw.rect(win, (0, 0, 255), (xoi, yoi - (yc - ys2), woi, hoi))   # connecting rod
            pygame.draw.rect(win, (0, 0, 255), (xp, yp - (yc - ys2), wp, hp))   # piston
            pygame.draw.polygon(win, (0, 255, 0), [[xua1, yua1], [xua2, yua2], [xua3, yua3]])   # arrow
        pygame.display.update()     # updating the display
        win.fill((0, 0, 0))         # filling the window with black
        clock.tick(FPS)             # limiting the frames per second, so the hardware wont be working too hard pointless


# Function which extracts data from configuration file and returns numbers for calculation:
def extract2calculate(name, n_str):     # function with two arguments: name of the file and number of line in file
                                        # to extract
    konfig = open(name, "r")            # opening the file
    s = []                              # creating empty list

    for line in konfig:                 # writing all characters in empty list with this cycle
        q = []
        for element in line:
            q += element
        s += [q]                        # - list with all characters

    q = []                              # reusing q list to write numbers from s list in it

    for i in range(len(s)):             # cycle to write numbers in q list
        for j in range(len(s[i])):
            q += ['']
            if s[i][j].isdigit() or (s[i][j] == "."):
                q[i] += s[i][j]

    return float(q[n_str])              # function returning the numbers in float format


def plot_viz(lock, payload, vel, reference):
    y = [0]
    x = [0]
    ys1 = [0]
    ys2 = [0]
    xs1 = [0]                           # problem on RPi may be here
    xs2 = [0]                           # problem on RPi may be here
    while alive:
        a = time.time()
        with lock:
            if payload['direction'] == 1:
                if y[-1] >= stroke - vel:
                    y += [stroke]
                    x += [a - reference]
                else:
                    y += [y[-1] + vel]
                    x += [a - reference]
            elif payload['direction'] == -1:
                if y[-1] > 0 + vel:
                    y += [y[-1] - vel]
                    x += [a - reference]
                else:
                    y += [0]
                    x += [a - reference]

            if payload['switch1']:
                ys1 += [1]
                xs1 += [a - reference]
            else:
                ys1 += [0]
                xs1 += [a - reference]
            if payload['switch2']:
                ys2 += [1]
                xs2 += [a - reference]
            else:
                ys2 += [0]
                xs2 += [a - reference]
            
        plt.subplot(311)
        plt.plot(x, y, '-ok')
        plt.ylabel('x, piston [m]')
        plt.subplot(312)
        plt.plot(x, ys1, '-ok')
        plt.ylabel('1. switch [-]')
        plt.subplot(313)
        plt.plot(x, ys2, '-ok')
        plt.xlabel('time [s]')
        plt.ylabel('2. switch [-]')
        if (a-reference) > 20:
            del y[0]
            del x[0]
            del ys1[0]
            del ys2[0]
        plt.plot()
        plt.pause(0.25)
        plt.clf()                           # clears the figure

    # plt.show()


# MAIN CODE HERE:

reference = time.time()         # time reference for plot_viz function

alive = True        # main switch

payload = {                     # payload for the defining the state of the system
    "switch1": False,
    "switch2": False,
    "direction": 0              # 1 = positive, -1 = negative
}

config_file = "konfiguration.txt"               # name of configuration txt file, should be in the same folder

d = extract2calculate(config_file, 0)/1000          # diameter of the cylinder [m]
p = extract2calculate(config_file, 1)*10**5         # working pressure [Pa]
mu = extract2calculate(config_file, 2)              # friction coefficient [-]
alpha = extract2calculate(config_file, 3)           # mounting angle of the cylinder
m_m = extract2calculate(config_file, 4)             # mass, that cylinder moves [kg]
F_th = extract2calculate(config_file, 5)            # theoretical force of the cylinder [N]
s_s = extract2calculate(config_file, 6)             # sensor width area
stroke = extract2calculate(config_file, 7)/1000     # stroke of the cylinder


A = (math.pi*d**2)/4                    # area of the piston
acc = (p*A)/m_m                         # acceleration of the piston
tc = math.sqrt((2*stroke)/acc)          # time of the cycle
vel = tc * acc                          # velocity of the piston
r = (2*s_s)/vel                         # reaction time of the sensors
F_b = mu*m_m*9.81*math.cos(alpha)       # static friction force

if F_th < F_b:                          # condition: if moving mass is too big
    vel = 0
    tc = 100000000
    r = 10000000

print(f'Calculated time of the cycle: {tc} [s]')
print(f'Calculated velocity of the piston: {vel} [m/s]')
print(f'Calculated reaction time of the sensors: {r} [s]')

if DEBUG:
    simulation_part = dummy_simulation_part     # for simulation on PC

# setting up threading:
lock = threading.Lock()
thread_simulation = threading.Thread(target=simulation_part, args=[lock, payload])
thread_vizualization = threading.Thread(target=vizualization_part, args=[lock, payload])
thread_plot = threading.Thread(target=plot_viz, args=[lock, payload, vel, reference])

thread_simulation.start()
thread_vizualization.start()
thread_plot.start()

print('starting the process')
# main switcher loop:
while True:
    try:
        time.sleep(0.01)
    except KeyboardInterrupt:
        print('Total Stop')
        alive = False
        sys.exit()
