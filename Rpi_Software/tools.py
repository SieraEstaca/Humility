from time import time, sleep

def Timer(period, start_time):
	elapsed_time = time() - start_time
	if elapsed_time < period:
		pause = period - elapsed_time
		sleep(pause)
	
			
def Record():
	fichier = open('data.dat','w')
	Ax = str(self.Ax)
	Ay = str(self.Ay)
	fichier.write(Ax)
	fichier.write(',')
	fichier.write(Ay)
	fichier.write(',')
	fichier.write('\n')
	sleep(1)	
	fichier.close()