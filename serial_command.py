#%% Imports
import serial


#%% Initializations

Serial_Port = "COM8"
Baud_Rate = 115200

SER = serial.Serial( Serial_Port, Baud_Rate)

#%% Main Run
if __name__ == "__main__":
    SER.flushInput()

    SER.write(b'\xff\xff\x02\x01\x06*\xc8\xc8')
    
    #SER.read_until(terminator=LF)

    SER.close()