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
res_1 = []
res_2 = []
res_3 = []
res_4 = []

err = []

for angle in circle:
    dist = 20
    source_pos = np.array([dist*np.cos(angle), dist*np.sin(angle)])

    dist_1 = la.norm(source_pos-mic_1_pos)
    dist_2 = la.norm(source_pos-mic_2_pos)
    dist_3 = la.norm(source_pos-mic_3_pos)
    dist_4 = la.norm(source_pos-mic_4_pos)

    phase_1 = 0
    phase_2 = dist_1 - dist_2
    phase_3 = dist_1 - dist_3
    phase_4 = dist_1 - dist_4


    lin = np.linspace(0, 200, 1024)

    mic_1 = np.sin(lin + phase_1) + np.random.normal(0.1, 0.1, 1024)
    mic_2 = np.sin(lin + phase_2) + np.random.normal(0.1, 0.1, 1024)
    mic_3 = np.sin(lin + phase_3) + np.random.normal(0.1, 0.1, 1024)
    mic_4 = np.sin(lin + phase_4) + np.random.normal(0.1, 0.1, 1024)

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

    index = approx_angle(np.array([mic_1_arg-mic_2_arg, mic_1_arg-mic_3_arg, mic_1_arg-mic_4_arg]))

    e = circle[index] - angle
    err.append(e)

    #res_1.append(mic_1_arg)
    res_2.append(mic_1_arg-mic_2_arg)
    res_3.append(mic_1_arg-mic_3_arg)
    res_4.append(mic_1_arg-mic_4_arg)


plt.plot(circle, err)

plt.figure()

#plt.plot(circle, res_1)
plt.plot(circle, res_2)
plt.plot(circle, res_3)
plt.plot(circle, res_4)

plt.show()

#print('base_1 = {}'.format(res_1))
print('base_2 = {}'.format(res_2))
print('base_3 = {}'.format(res_3))
print('base_4 = {}'.format(res_4))

while 1:

    angle = float(input('angle:'))

    dist = 20

    source_pos = np.array([dist*np.cos(angle), dist*np.sin(angle)])

    dist_1 = la.norm(source_pos-mic_1_pos)
    dist_2 = la.norm(source_pos-mic_2_pos)
    dist_3 = la.norm(source_pos-mic_3_pos)
    dist_4 = la.norm(source_pos-mic_4_pos)

    phase_1 = 0
    phase_2 = dist_1 - dist_2
    phase_3 = dist_1 - dist_3
    phase_4 = dist_1 - dist_4


    lin = np.linspace(0, 200, 1024)

    mic_1 = np.sin(lin + phase_1) + np.random.normal(0.1, 0.1, 1024)
    mic_2 = np.sin(lin + phase_2) + np.random.normal(0.1, 0.1, 1024)
    mic_3 = np.sin(lin + phase_3) + np.random.normal(0.1, 0.1, 1024)
    mic_4 = np.sin(lin + phase_4) + np.random.normal(0.1, 0.1, 1024)

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

    index = approx_angle(np.array([mic_1_arg-mic_2_arg, mic_1_arg-mic_3_arg, mic_1_arg-mic_4_arg]))

    print('angle: {}'.format(circle[index]))
    print(np.array([mic_1_arg, mic_2_arg, mic_3_arg, mic_4_arg]))



