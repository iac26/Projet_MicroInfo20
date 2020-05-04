"""
============
Oscilloscope
============

Emulates an oscilloscope.
"""

import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import threading
import struct

data_lr = [0]
data_fb = [0]
data_i = [0]

class Reader(threading.Thread):
    def __init__(self, com, baud):
        threading.Thread.__init__(self)
        self.ser = serial.Serial(com, baud)

    def run(self):
        cnt = 0
        while 1:
            state = 0

            while(state != 5):

                #reads 1 byte
                c1 = self.ser.read(1)
                #timeout condition
                if(c1 == b''):
                    print('Timout...')
                    return [];

                if(state == 0):
                    if(c1 == b'S'):
                        state = 1
                    else:
                        state = 0
                elif(state == 1):
                    if(c1 == b'T'):
                        state = 2
                    elif(c1 == b'S'):
                        state = 1
                    else:
                        state = 0
                elif(state == 2):
                    if(c1 == b'A'):
                        state = 3
                    elif(c1 == b'S'):
                        state = 1
                    else:
                        state = 0
                elif(state == 3):
                    if(c1 == b'R'):
                        state = 4
                    elif (c1 == b'S'):
                        state = 1
                    else:
                        state = 0
                elif(state == 4):
                    if(c1 == b'T'):
                        state = 5
                    elif (c1 == b'S'):
                        state = 1
                    else:
                        state = 0
            lr = struct.unpack('<f', self.ser.read(4))[0]
            fb = struct.unpack('<f', self.ser.read(4))[0]
            i = struct.unpack('<h', self.ser.read(2))[0]
            print("{}: {} || {}:{}".format(cnt,i, lr, fb))
            cnt+=1
            data_lr.append(lr)
            data_fb.append(fb)



class Scope(object):
    def __init__(self, ax, maxt=2, dt=0.02):
        self.ax = ax
        self.dt = dt
        self.maxt = maxt
        self.tdata = [0]
        self.ydata = [0]
        self.line = Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.set_ylim(-1.1, 1.1)
        self.ax.set_xlim(0, self.maxt)

    def update(self, y):
        lastt = self.tdata[-1]
        if lastt > self.tdata[0] + self.maxt:  # reset the arrays
            self.tdata = [self.tdata[-1]]
            self.ydata = [self.ydata[-1]]
            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
            self.ax.figure.canvas.draw()

        t = self.tdata[-1] + self.dt
        self.tdata.append(t)
        self.ydata.append(y)
        self.line.set_data(self.tdata, self.ydata)
        return self.line,


def data_lr_getter():
    while 1:
        if len(data_lr) > 1:
            yield data_lr.pop(0)
        else :
            yield data_lr[0]


def data_fb_getter():
    while 1:
        if len(data_fb) > 1:
            yield data_fb.pop(0)
        else :
            yield data_fb[0]




fig, ax = plt.subplots()
scope = Scope(ax)
fig.suptitle('FRONT_BACK', fontsize=16)

red = Reader("com5", 115200)

red.start()

# pass a generator in "emitter" to produce data for the update func
ani = animation.FuncAnimation(fig, scope.update, data_fb_getter, interval=10,
                              blit=True)

fig2, ax2 = plt.subplots()
scope2 = Scope(ax2)
fig2.suptitle('LEFT_RIGHT', fontsize=16)
# pass a generator in "emitter" to produce data for the update func
ani2 = animation.FuncAnimation(fig2, scope2.update, data_lr_getter, interval=10,
                              blit=True)

plt.show()
