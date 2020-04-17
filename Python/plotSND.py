import numpy as np
import serial
import sys
import time
import struct
from threading import Thread
import matplotlib.pyplot as plt


data = []


lin = np.linspace(0, 1024, 1024)
sin = np.sin(0.1*lin)
cos = np.cos(0.1*lin)

print('float val1[] = {', end='')
for i, d in enumerate(sin):
    print(' {}, {}'.format(d, cos[i]), end='')
    if i == 1023:
        print('', end='')
    else:
        print(',', end='')
    if (i % 10) == 0:
        print('')
print('};')

sin = np.sin(0.1*lin+2)
cos = np.cos(0.1*lin+2)

print('float val2[] = {', end='')
for i, d in enumerate(sin):
    print(' {}, {}'.format(d, cos[i]), end='')
    if i == 1023:
        print('', end='')
    else:
        print(',', end='')
    if (i % 10) == 0:
        print('')
print('};')

sin = np.sin(0.1*lin+4)
cos = np.cos(0.1*lin+4)

print('float val3[] = {', end='')
for i, d in enumerate(sin):
    print(' {}, {}'.format(d, cos[i]), end='')
    if i == 1023:
        print('', end='')
    else:
        print(',', end='')
    if (i % 10) == 0:
        print('')
print('};')


class serial_thread(Thread):

    # init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.contReceive = True
        self.alive = True
        self.need_to_update = False

        print('Connecting to port {}'.format(port))

        try:
            self.port = serial.Serial(port, timeout=1, baudrate=115200)
            print("connected")
            print(self.port)
        except:
            print('Cannot connect to the e-puck2')
            sys.exit(0)

    # function called after the init
    def run(self):
        while (self.alive):
            if (self.contReceive):
                global data
                data = []
                data.append(readFloatSerial(self.port))
                data.append(readFloatSerial(self.port))
                data.append(readFloatSerial(self.port))
                if len(data[0]) == 2*1024 and len(data[1]) == 2*1024 and len(data[2]) == 2*1024:
                    mic_1_plot.set_ydata(re(data[0]))
                    mic_2_plot.set_ydata(re(data[1]))
                    mic_3_plot.set_ydata(re(data[2]))
                    fft_1_plot.set_ydata(im(data[0]))
                    fft_2_plot.set_ydata(im(data[1]))
                    fft_3_plot.set_ydata(im(data[2]))

                    buf = bytes()
                    for dat in data:
                        for d in dat:
                            buf += struct.pack('f', d)

                    f.write(str(buf))

                    fft_graph.autoscale()
                    mic_graph.autoscale()
                    fft_graph.relim()
                    mic_graph.relim()


                self.tell_to_update_plot()
                #print(data)
            else:
                # flush the serial
                self.port.read(self.port.inWaiting())
                time.sleep(0.1)

    # clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if (self.port.isOpen()):
            while (self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()

    def plot_updated(self):
        self.need_to_update = False

    #tell if the plot need to be updated
    def need_to_update_plot(self):
        return self.need_to_update

    def tell_to_update_plot(self):
        self.need_to_update = True

def re(data):
    tmp = []
    for i, d in enumerate(data):
        if (i % 2) == 0:
            tmp.append(d)
    return tmp

def im(data):
    tmp = []
    for i, d in enumerate(data):
        if (i % 2) == 1:
            tmp.append(d)
    return tmp

# reads the FFT in float32 from the serial
def readFloatSerial(port):
    state = 0

    while (state != 5):

        # reads 1 byte
        c1 = port.read(1)
        # timeout condition
        if (c1 == b''):
            print('Timout...')
            return [];

        if (state == 0):
            if (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif (state == 1):
            if (c1 == b'T'):
                state = 2
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif (state == 2):
            if (c1 == b'A'):
                state = 3
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif (state == 3):
            if (c1 == b'R'):
                state = 4
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif (state == 4):
            if (c1 == b'T'):
                state = 5
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0

    # reads the size
    # converts as short int in little endian the two bytes read
    size = struct.unpack('<h', port.read(2))
    # removes the second element which is void
    size = size[0]


    # reads the data
    rcv_buffer = port.read(size * 4)
    print("size: {} / {}".format(size, len(rcv_buffer)))
    data = []

    # if we receive the good amount of data, we convert them in float32
    if (len(rcv_buffer) == 4 * size):
        i = 0
        while (i < size):
            data.append(struct.unpack_from('<f', rcv_buffer, i * 4)[0])
            i = i + 1

        #print(data)
        print('received ')
        return data
    else:
        print('Timout...')
        return []

def update_plot_mic():
    pass

def update_plot():
    if (reader.need_to_update_plot()):
        fig.canvas.draw_idle()
        reader.plot_updated()

def handle_close(evt):
    reader.stop()
    f.close()

def do_fft(array):
    FFT = np.fft.fft(array)
    FFT_norme = np.sqrt(np.add(np.multiply(np.real(FFT),np.real(FFT)),(np.multiply(np.imag(FFT),np.imag(FFT)))))
    return FFT_norme

com = input("Enter com port: ")

f = open('save.dat', 'w+')

fig, ax = plt.subplots(num=None, figsize=(10, 8), dpi=80)
fig.canvas.set_window_title('Noisy plot')
plt.subplots_adjust(left=0.1, bottom=0.25)
fig.canvas.mpl_connect('close_event', handle_close)

mic_graph = plt.subplot(211)
mic_1_plot, = plt.plot(np.linspace(0, 6, 1024), lw=1, color='red')

mic_2_plot, = plt.plot(np.linspace(3, 6, 1024), lw=1, color='green')

mic_3_plot, = plt.plot(np.linspace(0, 3, 1024), lw=1, color='blue')

fft_graph = plt.subplot(212)
fft_1_plot, = plt.plot(np.arange(-512,512,1), do_fft(np.linspace(0, 6, 1024)), lw=1, color='red')

fft_2_plot, = plt.plot(np.arange(-512,512,1), do_fft(np.linspace(3, 6, 1024)), lw=1, color='green')

fft_3_plot, = plt.plot(np.arange(-512,512,1), do_fft(np.linspace(0, 3, 1024)), lw=1, color='blue')

reader = serial_thread(com)
reader.start()

timer = fig.canvas.new_timer(interval=50)
timer.add_callback(update_plot)
timer.start()

plt.show()

