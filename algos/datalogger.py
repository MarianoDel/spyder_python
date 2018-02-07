import serial
import sys
import os
import threading
import time
from datetime import datetime


port = 'COM20'
baud = 9600

def SerialWorker (s):
    print ("In s worker",flush=True)
    i = 0
    while True:
#        """Pruebas Rx"""
#        time.sleep(5)
#        print("worker s", flush=True)
        # bytes = s.in_waiting
        # if (bytes>0):
        #     reading = s.read(bytes).decode()
        #     print("rx bytes: " + str(bytes) + " -> " + reading,flush=True)

        # rx_bytes = s.in_waiting
        # if (rx_bytes>0):
        #     data = s.read(rx_bytes).decode()
        #     print ("Rx: " + data,flush=True)
        """Pruebas Tx"""
        time.sleep(10)
        data_to_send = "prueba i = " + str(i) + "\n"
        n = s.write(data_to_send.encode('utf-8'))
        print ("Envie " + str(n) + " bytes",flush=True)
        i += 1
        
    
    
    
#serial_port = serial.Serial(port, baud, timeout=0)
serial_string = ""
start_time = time.time()

def MadeString(data):
    """ Entra un solo byte o string, reviso si es fin de linea"""
    global serial_string
    global start_time
    for d in data:
        if (d == '\r'):                #si llega \r no hago nada
            continue
        if (d != '\n'):
            serial_string += d
#            print(d,flush=True)
        else:
            elapsed_time = time.time() - start_time
# #            last_time = time.time()             #solo si quiero segundos delta
            elapsed_int_time = int(elapsed_time)
            #armo el string para el log
            ser = str(elapsed_int_time) + ',' + serial_string            
            print (ser,flush=True)
            WriteLog(ser)
            serial_string = ""

            
    
    
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

def WriteLog (data):
    with open('logger.txt', 'a') as f:
        f.write(str(data) + '\n')

def FlushLog (data):
    with open('logger.txt', 'w') as f:
        f.write(str(data) + '\n')
        
# with open('logger.txt', 'w') as f:
    # f.write(str(data))

#---- MAIN FUNC ----#
def Main():    
    try:
        serial_port = serial.Serial(port, baud, timeout=None)              #blocking mode
        print("Arrancando en puerto " + port + " velocidad " + str (baud))

        # st = threading.Thread(target = SerialWorker, args=[serial_port])
        # print("Inicio Thread",flush=True)
        # st.start()

#        f = open('logger.txt', 'w')
        start = str(datetime.now())
        FlushLog(start)

#        if True:
        while True:
            rx_bytes = serial_port.in_waiting
            if (rx_bytes>0):
                data = serial_port.read(rx_bytes).decode()
                MadeString(data)
#                print("Rx: " + data + '\n',flush=True)
                

        
        


        # while True:
        #     time.sleep(3)
        #     data = 'hello\n'.encode('utf-8')
        #     print (data,flush=True)
        #     n = serial_port.write(data)
        #     print ("Envie " + str(n) + " bytes",flush=True)

        #     #Reviso si tengo la linea del puerto serie
        #     new_line = GetSerial()
        #     if (new_line != None):
        #         print("Rx: " + new_line,flush=True)
                
            

            # key = anykey()
            # if (key != None):
            #     handle_data(key, True)
            #     serial_port.write(key)



    except:
        print("No se encontro puerto " + port)
        
        

    
if __name__ == '__main__':
    Main()


