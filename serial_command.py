#%% Imports
import serial
import time

#%% Initializations

Serial_Port = "COM8"
Baud_Rate = 115200

SER = serial.Serial( Serial_Port, Baud_Rate)

#%% Main Run
if __name__ == "__main__":
    SER.flushInput()
    
    # Clear LCD
    SER.write(b'\xff\xff\x02***\xc8\xc8')
    
    time.sleep(1)
    # LCD Goto (Row, Col)
    row, col = (2, 4)
    tmp_packet = '\xff\xff\x03{}{}*\xc8\xc8'.format(chr(row), chr(col))
    SER.write(bytes(tmp_packet, encoding='ANSI'))
    
    time.sleep(1)
    # Send a string to show on LCD
    for char_i in 'Parham Joon!':        
        tmp_packet = '\xff\xff\x04{}\x06*\xc8\xc8'.format(char_i)
        SER.write(bytes(tmp_packet, encoding='ANSI'))
        time.sleep(2000/Baud_Rate)

    time.sleep(1)
    # LCD Shift Entire Display to the Left
    SER.write(b'\xff\xff\x05\x18**\xc8\xc8')

    

    SER.close()