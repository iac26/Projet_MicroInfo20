import numpy as np
import serial
import struct
import cv2
import time



#maximum value for an uint8
max_value = 255


    



#reads the data in uint8 from the serial
def readUint8Serial(port):

    state = 0

    while(state != 5):

        #reads 1 byte
        c1 = port.read(1)
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

    #reads the size
    #converts as short int in little endian the two bytes read
    size = struct.unpack('<h',port.read(2)) 
    visible = struct.unpack('<B',port.read(1))[0]
    center = struct.unpack('<h',port.read(2))[0]
    width = struct.unpack('<h',port.read(2))[0] 
    #removes the second element which is void
    size = size[0]  

    #reads the data
    rcv_buffer = port.read(size)
    data = []

    if(len(rcv_buffer) == size):
        i = 0
        while(i < size):
            data.append(struct.unpack_from('<B',rcv_buffer, i)[0])
            i = i + 1

        #print('received {} bytes !'.format(size))
        return data, visible, center, width
    else:
        print('Timout...')
        return [], 0, 0, 0

try:
	reader = serial.Serial('com5', 115200)
except:
	print('error')
	time.sleep(1)
	quit()

def analyse_col(x):
	EDGE_T = 80
	PATTERN_TOL = 4
	NB_TOL = 2
	last_rise = 0
	last_fall = 0
	rise_distances = list(range(20))
	fall_distances = list(range(20))
	r2f_distances = list(range(20))
	f2r_distances = list(range(20))
	nb_rises = 0
	nb_falls = 0
	nb_r2f = 0
	nb_f2r = 0
	y = 0
	while y < height-1:
		#look for rising edges
		a = int(image[y][x]) 
		b = int(image[y+1][x])
		if (a - b) < -EDGE_T:
			if nb_rises == 0:
				last_rise = y
				nb_rises += 1 
				if nb_falls != 0:
					f2r_distances[nb_f2r] = last_rise- last_fall
					nb_f2r += 1
			else:
				rise_distances[nb_rises-1] = y - last_rise
				nb_rises += 1
				last_rise = y
				if nb_falls != 0:
					f2r_distances[nb_f2r] = last_rise- last_fall
					nb_f2r += 1 

			rising_heights.append((x, y))
		if (a - b) > EDGE_T:
			if nb_falls == 0:
				last_fall = y
				nb_falls += 1
				if nb_rises != 0:
					r2f_distances[nb_r2f] = last_fall - last_rise
					nb_r2f += 1
			else:
				fall_distances[nb_falls-1] = y - last_fall
				nb_falls += 1
				last_fall = y
				if nb_rises != 0:
					r2f_distances[nb_r2f] = last_fall - last_rise
					nb_r2f += 1

			falling_heights.append((x, y))
		y += 1
	fall_moy = 0
	rise_moy = 0
	r2f_moy = 0 
	f2r_moy = 0 
	if nb_rises > 1 and nb_falls > 1 and nb_r2f > 0 and nb_f2r > 0:
		for i in range(nb_falls-1):
			fall_moy += fall_distances[i]
		fall_moy /= nb_falls-1

		for i in range(nb_rises-1):
			rise_moy += rise_distances[i]
		rise_moy /= nb_rises-1

		for i in range(nb_r2f):
			r2f_moy += r2f_distances[i]
		r2f_moy /= nb_r2f 

		for i in range(nb_f2r):
			f2r_moy += f2r_distances[i]
		f2r_moy /= nb_f2r 


		#print('fm:{} rm: {} r2f:{} f2r: {}'.format(fall_moy, rise_moy, r2f_moy, f2r_moy))

		if abs(fall_moy - rise_moy) < PATTERN_TOL and abs(r2f_moy/2 - f2r_moy) < PATTERN_TOL and nb_rises > NB_TOL:
			pattern_col.append(x) 
			return True
	return False



width = 160
height = 40
while True:
	prev = time.time()
	data, v, c, w = readUint8Serial(reader)
	print(v, c, w)
	image = np.array(data, dtype=np.uint8).reshape(height, width)


	rising_heights = []
	falling_heights = []
	pattern_col = []

	SUB_MAX = 16
	
	subdivision = 2
	larg = width
	found = 0
	while subdivision <= SUB_MAX and not found:
		prev_larg = larg
		larg = width/subdivision
		col = larg
		while col < width and not found:
			if col % prev_larg != 0:
				if analyse_col(int(col)):
					found = col
			col += larg
		subdivision *= 2

	exploration_step = 4
	max_fails = 4
	if found:
		last_col = found
		first_col = found

		col = found+exploration_step
		fails = 0
		while col < width:
			
			if analyse_col(int(col)):
				last_col = col
				fails = 0
			else:
				fails += 1
				if fails > max_fails:
					break
			col += exploration_step

		col = found-exploration_step
		fails = 0
		while col >= 0:
			
			if analyse_col(int(col)):
				first_col = col
				fails = 0
			else:
				fails += 1
				if fails > max_fails:
					break
			col -= exploration_step
		pos = (first_col + last_col)/2
		largeur = last_col - first_col
		print('first: {}, last: {} || pos: {} width: {}'.format(first_col, last_col, pos, largeur))







			






	scaled = cv2.resize(image, (width*10, height*10))
	scaled = cv2.cvtColor(scaled, cv2.COLOR_GRAY2BGR)

	for r in rising_heights:
		scaled = cv2.line(scaled, (10*r[0]-50, r[1]*10), (10*r[0]+50, r[1]*10), (255, 0, 0), 1)
	for f in falling_heights:
		scaled = cv2.line(scaled, (10*f[0]-50, f[1]*10), (10*f[0]+50, f[1]*10), (0, 0, 255), 1)

	for p in pattern_col:
		scaled = cv2.line(scaled, (10*p, 0), (10*p, 40*10), (0, 255, 0), 1)

	if v:
		scaled = cv2.line(scaled, (10*int(c-w/2), 0), (10*int(c-w/2), 40*10), (255, 0, 255), 3)
		scaled = cv2.line(scaled, (10*int(c+w/2), 0), (10*int(c+w/2), 40*10), (255, 0, 255), 3)
		scaled = cv2.line(scaled, (10*int(c), 0), (10*int(c), 40*10), (255, 0, 128), 6)

	cv2.imshow('captured_image', scaled)
	#cv2.imwrite("Image_1.png", image)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	print('FPS: {}'.format(1/(time.time()-prev)))

cv2.destroyAllWindows()
