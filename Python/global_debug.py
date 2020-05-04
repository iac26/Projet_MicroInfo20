import socket, threading
import serial


HOST = ''
PORT = 23 

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(4)
clients = [] #list of clients connected
lock = threading.Lock()


class broadCaster(threading.Thread):
	def __init__(self, port, baud):
		threading.Thread.__init__(self)
		self.ser = serial.Serial(port, baud)
		print(self.ser)


	def run(self):
		while True:
			data = self.ser.read(16)
			print(data.decode(), end='')
			lock.acquire()
			for c in clients:
				c.socket.send(data)
			lock.release()


class chatServer(threading.Thread):
    def __init__(self, soc, addr):
        threading.Thread.__init__(self)
        self.socket = soc
        self.address = addr

    def run(self):
        lock.acquire()
        clients.append(self)
        lock.release()
        print('%s:%s connected.' % self.address)
        while True:
            data = self.socket.recv(1024)
            if not data:
                break
            print('recieved from {}: {}'.format(self.address, data))
        self.socket.close()
        print('%s:%s disconnected.' % self.address)
        lock.acquire()
        clients.remove(self)
        lock.release()

broadCaster('com5', 115200).start()

while True:  # wait for socket to connect
    
    # send socket to chatserver and start monitoring
    chatServer(*s.accept()).start()
    
