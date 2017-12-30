import serial
import sys
import os
import threading
import time


port = 'COM20'
baud = 9600

def SerialWorker (s):
    print ("In s worker",flush=True)
    while True:
        time.sleep(5)
        data = 'hello\n'.encode('utf-8')
        print (data,flush=True)
        n = s.write(data)
        print ("Envio s worker" + str(n) + " bytes",flush=True)
    
    
    
#serial_port = serial.Serial(port, baud, timeout=0)

def handle_data(data, console=False):
    if (console):
        print("Console: " + data)
    else:
        print("Rx: " + data)


def anykey():

   try:
       # ch = os.read(sys.stdin.fileno(), 1)
       np = input('Choose a number: ')
   except:
       np = None
       print ("Not a char or Number")
       
#       pass

   return np

# with open('logger.txt', 'w') as f:
    # f.write(str(data))

#---- MAIN FUNC ----#
def Main():
    try:
        serial_port = serial.Serial(port, baud, timeout=None)              #blocking mode
        print("Arrancando en puerto " + port + " velocidad " + str (baud))

        st = threading.Thread(target = SerialWorker, args=[serial_port])
        print("Inicio Thread",flush=True)
        st.start()
        


        while True:
            # time.sleep(3)

            bytes = serial_port.in_waiting
            if (bytes>0):
                reading = serial_port.read(bytes).decode()
                print("rx bytes: " + str(bytes) + " -> " + reading,flush=True)

            # key = anykey()
            # if (key != None):
            #     handle_data(key, True)
            #     serial_port.write(key)



    except:
        print("No se encontro puerto " + port)
        
        

    
if __name__ == '__main__':
    Main()
