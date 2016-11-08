
#----------------------------NIET AANPASSEN------------------------------
import imp
import can_lib_auguste as au
#au = imp.load_source('module.name', '/home/pi/spi_auguste/spi_can/can_lib_auguste.py')

au.startSpi(500000, 0)

CNF1 = 0x0F 
CNF2 = 0x90
CNF3 = 0x02

au.softReset()
au.setCANCTRL(0x80) #set configuration mode
au.setCANINTE(0x0E) #enable interrupts on transmit empty and on receive full
au.extendedID()        #enable extended identifier
au.setCANINTF(0x00) #clear all interrupt flags
au.setRXBnCTRL(0x64)    #accept all incomming messages and enable roll over
au.setCNF1(CNF1)    #Used to be:0x0F 
au.setCNF2(CNF2)    #Used to be:0x90
au.setCNF3(CNF3)    #Used to be:0x02

au.setTXBnSIDH(0x00, 0) #set standard identifier 8 high bits
au.setTXBnSIDL(0x08, 0) #set low 3 bits stid and extended identifier
au.setTXBnEID8(0x00, 0)
au.setTXBnEID0(0x02, 0)

au.setTXBnDLC(0x01, 0)  #Transmitted message will be a dataframe with 1 bits
au.setTXBnDM([3 for x in range(8)], 0)
au.setCANCTRL(0x00)


au.getTXBnDM()

#----------------------------WEL AANPASSEN------------------------------

import time
import numpy as np

delta_meting = 1 #tijd tussen elk meetpunt in seconden
num_loops = 60*60*4 #Hoeveel meetpunten wil je
naam_textfile = "test1" #naam van het bestand waar de test wordt naartoe geschreven
res_list = [] #lijst waar de resultaten in komen

c_time = 0 #niet aanpassen

for x in range(num_loops):
    c_time = time.time()
    au.setCANINTF(0x00)
    print(str(float(x)/(num_loops)*100)+"%")
    au.setTXBnCTRL(0x0B)
    volt = au.getVoltage()
    res_list.append(volt)
    while (time.time() < c_time + delta_meting):
        pass

#dit schrijft alles weg naar tekstfilenaam, niet aanpassen
with open("./testresultaten/" + naam_textfile, "w") as text_file:
    for x in range(num_loops):
        text_file.write("%.8f\n" %(res_list[x]))
print("DONE")

#import matplotlib.pyplot as plt

#plt.scatter(range(len(res_list)), res_list)
#plt.show()

