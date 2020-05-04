#research for sound localisation
import numpy as np
from numpy import linalg as la
import matplotlib.pyplot as plt



#returns a list of peak indexes
def peak_detect(array):
    i = 1
    res = []
    while i < len(array)-1:
        if array[i] > array[i-1] and array[i] > array[i+1]:
            res.append(i)
        i += 1
    return res

def do_sinus(freq, phase):
    n = 1024  # nb of samples
    fs = 300  # sampling frequency
    m = np.linspace(0, n, num=(fs*n))
    sinus = np.sin(2*np.pi*freq*m + phase)
    sinus = sinus[0:n]
    return sinus

def approx_angle(mics):
    i = 0
    min = 99999
    ix = 0
    pl = []

    while i < 1024:
        ref = np.array([base_2[i], base_3[i], base_4[i]])
        dst = la.norm(mics-ref)
        #print('i={}, dst={}'.format(i, dst))
        pl.append(dst)
        if dst < min:
            min = dst
            ix = i
        i += 1

    return ix



mic_1_pos = np.array([0, 1])
mic_2_pos = np.array([-0.7, -0.3])
mic_3_pos = np.array([0.7, -0.3])
mic_4_pos = np.array([0, -1])

circle = np.linspace(-np.pi, np.pi, 1024)
mic_1 = []
mic_2 = []
mic_3 = []
mic_4 = []
#simulates sound from a direction and fills the buffers: res_[1-4]
def sim_sound(angle):
    global mic_1
    global mic_2
    global mic_3
    global mic_4
    source_pos = np.array([dist * np.cos(angle), dist * np.sin(angle)])

    dist_1 = la.norm(source_pos - mic_1_pos)
    dist_2 = la.norm(source_pos - mic_2_pos)
    dist_3 = la.norm(source_pos - mic_3_pos)
    dist_4 = la.norm(source_pos - mic_4_pos)

    phase_1 = 0
    phase_2 = dist_1 - dist_2
    phase_3 = dist_1 - dist_3
    phase_4 = dist_1 - dist_4

    lin = np.linspace(0, 200, 1024)

    mic_1 = np.sin(lin + phase_1) + np.random.normal(0.1, 0.1, 1024)
    mic_2 = np.sin(lin + phase_2) + np.random.normal(0.1, 0.1, 1024)
    mic_3 = np.sin(lin + phase_3) + np.random.normal(0.1, 0.1, 1024)
    mic_4 = np.sin(lin + phase_4) + np.random.normal(0.1, 0.1, 1024)


def calc_data_max():
    mic_1_fft = np.fft.fft(mic_1)
    mic_2_fft = np.fft.fft(mic_2)
    mic_3_fft = np.fft.fft(mic_3)
    mic_4_fft = np.fft.fft(mic_4)

    mic_1_imag = np.max(np.imag(mic_1_fft))
    mic_1_real = np.max(np.real(mic_1_fft))
    mic_2_imag = np.max(np.imag(mic_2_fft))
    mic_2_real = np.max(np.real(mic_2_fft))
    mic_3_imag = np.max(np.imag(mic_3_fft))
    mic_3_real = np.max(np.real(mic_3_fft))
    mic_4_imag = np.max(np.imag(mic_4_fft))
    mic_4_real = np.max(np.real(mic_4_fft))

    mic_1_arg = np.arctan2(mic_1_imag, mic_1_real)
    mic_2_arg = np.arctan2(mic_2_imag, mic_2_real)
    mic_3_arg = np.arctan2(mic_3_imag, mic_3_real)
    mic_4_arg = np.arctan2(mic_4_imag, mic_4_real)

    return (mic_1_arg, mic_2_arg, mic_3_arg, mic_4_arg)



circle = np.linspace(-np.pi, np.pi, 1024)



sin = do_sinus(1, 0)

plt.plot(sin)

plt.show()


