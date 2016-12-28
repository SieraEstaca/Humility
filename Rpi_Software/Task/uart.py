import serial
from time import time
from tools import Timer

class Arduino():

	def __init__(self, period):
		# Start serial
		self.period = period
		self.sensorsData = serial.Serial(
			port = '/dev/ttyACM0', 
			baudrate = 9600, 
			timeout = 0.25)

		# getDatas params
		self.val1 = 0.0
		self.val2 = 0.0
		self.val3 = 0.0
		self.val4 = 0.0

	def sendDatas(self, val_a, val_b):
		start_time = time()

		# Send datas
		send_dummy = str(val_a) + ',' + str(val_b) + ',' + "true" + '\n'
		self.sensorsData.flush()
		self.sensorsData.write(send_dummy)

		Timer(self.period, start_time)

	def getDatas(self):
		start_time = time()
		
		# Get Datas
		try:
			textline = self.sensorsData.readline()
			dataNums = textline.split(',')
			if len(dataNums)==4: 
				self.val1 = float(dataNums[0])
				self.val2 = float(dataNums[1])
				self.val3 = float(dataNums[2])
				self.val4 = float(dataNums[3])
		except ValueError:
			pass

		Timer(self.period, start_time)
		return self.val1, self.val2, self.val3, self.val4
