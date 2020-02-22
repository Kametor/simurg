import math
class PID :

	def __init__(self, kp, ki, kd, coz_x=640, coz_y=480, mid_hiz=0):

		self.kp = kp				# Hiz sabitlerini tanimla
		self.ki = ki
		self.kd = kd

		self.cozunurluk_x = coz_x   # Cozunurlugu tanimla
		self.cozunurluk_y = coz_y

		self.mid_hiz = mid_hiz		# Hiz baslangic hiz degerini tanimla
		self.perror = 0		    # Onceki hatayi tanimla
		self.errori = 0		    # Hatayi tanimla

	def update(self, ort_x, ort_y):

		error = ((ort_x - (self.cozunurluk_x/2.0))**2 + ((self.cozunurluk_y/2.0) - ort_y)**2)**0.5

		if (error + self.errori)*self.ki < 3 and (error + self.errori)*self.ki > -3:
			self.errori = error + self.errori

		errord = error - self.perror

		pid = self.kp*error + self.ki*self.errori + self.kd*errord

		if errord < 0 and pid > 0 :
			pid = pid + self.kd*errord

		elif errord > 0 and pid < 0:
			pid = pid + self.kd*errord

		elif errord > 0 and pid > 0:
			pid = pid + self.ki*self.errori

		elif errord < 0 and pid < 0:
			pid = pid + self.ki*self.errori

		hiz = self.mid_hiz + pid


		self.perror = error

		if hiz > 0.5:
			hiz = 0.5

		aci = math.atan2(self.cozunurluk_y/2-ort_y, ort_x-self.cozunurluk_x/2)	   #dronun kendisine gore hedefle yaptigi radyan cinsinden aci
		vx=hiz*math.sin(aci)    					   # x yonundeki hizi
		vy=hiz*math.cos(aci) 					   # y yonundeki hizi

		return (vx,vy,aci,error)

if __name__ == "__main__" :
	pid = PID(10, 20, 30)
	result = pid.update(600,20)
	print("HizX:{}, HizY:{}, Aci:{}".format(result[0], result[1], result[2]))
