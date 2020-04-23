import numpy as np
from numpy import linalg as la
import serial
import sys
import time
import struct
from threading import Thread
import matplotlib.pyplot as plt


data = []

reader_on = True


def do_fft(array):
    FFT = np.fft.fft(array)
    return FFT
def do_norm(FFT):
    FFT_norme = np.sqrt(np.add(np.multiply(np.real(FFT),np.real(FFT)),(np.multiply(np.imag(FFT),np.imag(FFT)))))
    return FFT_norme
def do_arg(FFT):
    FFT_arg = np.arctan2(np.imag(FFT), np.real(FFT))
    return FFT_arg

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


def find_args_FFT(data):
    res = []
    for dset in data:
        fft = do_fft(dset)
        norm = do_norm(fft)[512:]
        arg = do_arg(fft)[512:]
        max = 0
        ix = 0
        for i, e in enumerate(norm):
            if e > max:
                max = e
                ix = i
        print(ix)
        a = arg[ix]
        res.append(a)

    return res

def angulate(a):
    while a > np.pi:
        a -= np.pi * 2
    while a < -np.pi:
        a += np.pi * 2
    return a


class serial_thread(Thread):

    # init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.contReceive = True
        self.alive = True
        self.need_to_update = False
        self.port = None
        print(reader_on)
        if not reader_on:
            return

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
                if reader_on:
                    data.append(np.array(readFloatSerial(self.port)))
                    data.append(np.array(readFloatSerial(self.port)))
                    data.append(np.array(readFloatSerial(self.port)))
                    data.append(np.array(readFloatSerial(self.port)))
                    #print(data)
                else:
                    pass
                if len(data[0]) == 1024 and len(data[1]) == 1024 and len(data[2]) == 1024 and len(data[3]) == 1024:
                    #n1 = np.max(data[0])
                    #n2 = np.max(data[1])
                    #n3 = np.max(data[2])
                    #n4 = np.max(data[3])
                    #data[0] = data[0] / n1
                    #data[1] = data[1] / n2
                    #data[2] = data[2] / n3
                    #data[3] = data[3] / n4
                    mic_1_plot.set_ydata((data[0]))
                    mic_2_plot.set_ydata((data[1]))
                    mic_3_plot.set_ydata((data[2]))
                    mic_4_plot.set_ydata((data[3]))

                    res = find_args_FFT(data)

                    a1 = angulate(res[0] - res[1])
                    a2 = angulate(res[0] - res[2])
                    a3 = angulate(res[0] - res[3])


                    fft_1_plot.set_ydata(np.linspace(a1, a1, 5))
                    fft_2_plot.set_ydata(np.linspace(a2, a2, 5))
                    fft_3_plot.set_ydata(np.linspace(a3, a3, 5))

                    mic_graph.autoscale()
                    mic_graph.relim()


                self.tell_to_update_plot()
                #print(data)
            else:
                # flush the serial
                if reader_on:
                    self.port.read(self.port.inWaiting())
                time.sleep(0.1)

    # clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if reader_on:
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


com = input("Enter com port: ")

index = 0
rdata = []




fig, ax = plt.subplots(num=None, figsize=(10, 8), dpi=80)
fig.canvas.set_window_title('Noisy plot')
plt.subplots_adjust(left=0.1, bottom=0.25)
fig.canvas.mpl_connect('close_event', handle_close)

mic_graph = plt.subplot(211)
plt.grid()
mic_1_plot, = plt.plot(np.linspace(0, 6, 1024), lw=1, color='red')

mic_2_plot, = plt.plot(np.linspace(3, 6, 1024), lw=1, color='green')

mic_3_plot, = plt.plot(np.linspace(0, 3, 1024), lw=1, color='blue')

mic_4_plot, = plt.plot(np.linspace(0, 3, 1024), lw=1, color='orange')

fft_graph = plt.subplot(212)
fft_1_plot, = plt.plot(np.linspace(-np.pi, -np.pi, 5), lw=1, color='red')

fft_2_plot, = plt.plot(np.linspace(0, 0, 5), lw=1, color='green')

fft_3_plot, = plt.plot(np.linspace(np.pi, np.pi, 5), lw=1, color='blue')

fft_graph.autoscale()
fft_graph.relim()


timer = fig.canvas.new_timer(interval=50)
timer.add_callback(update_plot)
timer.start()

reader = serial_thread(com)
reader.start()


plt.show()

