import struct

import matplotlib.pyplot as mpl

import math


rdata = []

reader_on = False
f = open('save.bin', 'rb')
print(f)
end = False
while not end:
    tmp2 = []
    ix = 0
    while ix < 3:
        tmp = []
        size =2048
        try:
            raw = f.read(4 * size)
            print(len(raw))
            if (len(raw) == 4 * size):
                i = 0
                while (i < size):
                    tmp.append(struct.unpack_from('f', raw, i * 4)[0])
                    i = i + 1
            else:
                end = True
                break
        except:
            print('err')
            quit = True
            break
        print(tmp)
        tmp2.append(tmp)
        ix += 1
    rdata.append(tmp2)
print(rdata)


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


def norm(data):
    tmp = []
    i = 0
    while i < len(data)/2:
        tmp.append(math.sqrt(math.pow(data[2*i], 2)+math.pow(data[2*i+1], 2)))
        i += 1
    return tmp

def arg(data):
    tmp = []
    i = 0
    while i < len(data)/2:
        tmp.append(math.atan2(data[2*i+1], data[2*i]))
        i += 1
    return tmp


ix = 2
mpl.plot(arg(rdata[ix][0]))
mpl.plot(arg(rdata[ix][1]))
mpl.plot(arg(rdata[ix][2]))
#mpl.plot(re(rdata[0][0]))

mpl.show()

f.close()

f = open('data.csv', 'w+')


for line in rdata:
    for graph in line:
        for d in graph:
            f.write(str(d)+':')

        f.write('\n')
f.close()