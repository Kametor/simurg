# Bu kod Mehmet Recep Askar tarafından, 18.01.2020 tarihinde, 2019 Teknofest Savasan IHA yarismasinda kullanılan PID kontrolcusunun UORG Bilgisayarli Goru yarismasi icin yeniden düzenlenmis halidir

class PID :

	def __init__(self, kph, kih, kdh, coz_x=1920, coz_y=1080, mid_hiz=0):

		self.kph = kph				# Hiz sabitlerini tanimla
		self.kih = kih
		self.kdh = kdh

		self.cozunurluk_x = coz_x   # Cozunurlugu tanimla
		self.cozunurluk_y = coz_y

		self.mid_hiz = mid_hiz		# Hiz baslangic hiz degerini tanimla
		self.perrorh = 0		    # Onceki hatayi tanimla
		self.errorih = 0		    # Hatayi tanimla

	def update(self, sol_ustx, sol_usty, sag_altx, sag_alty):

		# Dikdortgenlerin diger koselerini tanimla
		sol_altx = sol_ustx
		sol_alty = sag_alty
		sag_ustx = sag_altx
		sag_usty = sol_usty

		# Kilitlenme dikdortgeninin merkez koordinatini hesapla
		ort_x = (sol_ustx+sag_altx)/2.0
		ort_y = (sol_usty+sag_alty)/2.0

		# Hiz icin PID hesapla
		error = ((ort_x - (self.cozunurluk_x/2.0))**2 + ((self.cozunurluk_y/2.0) - ort_y)**2)**0.5      # Hatayı hesapla

		if (error + self.errorih)*self.kih < 3 and (error + self.errorih)*self.kih > -3:
			self.errorih = error + self.errorih

		errordh = error - self.perrorh

		pidh = self.kph*error + self.kih*self.errorih + self.kdh*errordh

		if errordh < 0 and pidh > 0 :
			pidh = pidh + self.kdh*errordh

		elif errordh > 0 and pidh < 0:
			pidh = pidh + self.kdh*errordh

		elif errordh > 0 and pidh > 0:
			pidh = pidh + self.kih*self.errorih

		elif errordh < 0 and pidh < 0:
			pidh = pidh + self.kih*self.errorih

		#  Hiz degerlerini hesapla
		hiz = self.mid_hiz + pidh


		# Onceki error degerini guncelle
		self.perrorh = errorh

                if hiz > 3:
                    hiz = 3

		aci = atan2(coz_y-orty, ortx-coz_x)	   #dronun kendisine gore hedefle yaptigi radyan cinsinden aci
		vx=hiz*sin(aci)    					   # x yonundeki hizi
		vy=hiz*cos(aci) 					   # y yonundeki hizi

		return (vx,vy,aci)

# Birim testi
if __name__ == "__main__" :
	pid = PID(10, 20, 30, 10, 20, 30, 10, 20, 30)
	result = pid.update(50, 100, 70, 200)
	print("HizX:{}, HizY:{}, Aci:{}".format(result[0], result[1], result[2])
