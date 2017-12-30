import serial
import time


port = 'COM20'
baud = 9600


#---- MAIN FUNC ----#
def Main():
    try:
        serial_port = serial.Serial(port, baud, timeout=None)              #blocking mode
        print("Arrancando en puerto " + port + " velocidad " + str (baud))

#        if True:
        while True:
 
            time.sleep(3)
            data = 'hello'.encode('utf-8')
            print (data,flush=True)
            n = serial_port.write(data)
            print ("Envie " + str(n) + " bytes",flush=True)
            
            
            # bytes = serial_port.inWaiting()
            # if (bytes>0):
            #     reading = serial_port.read(bytes).decode()
            #     handle_data(reading)

            # key = anykey()
            # if (key != None):
            #     handle_data(key, True)
            #     serial_port.write(data)



    except:
       print("No se encontro puerto " + port)
        
        

    
if __name__ == '__main__':
    Main()


