import serial
import sys
import time


def init_vs2(ser:serial.Serial) -> bool:

    # after the serial port read buffer is emptied
    ser.reset_input_buffer()

    # then an EOT (0x04) is send
    ser.write([0x04])

    # and for 30x100ms waited for an ENQ (0x05)
    i = 0
    while(i < 30):
        time.sleep(0.1)
        buff = ser.read(1)
        print(buff)
        if(len(buff) > 0):
            if(int(buff[0]) == 0x05):
                break
        i+=1

    if(i == 30):
        return False
    
    ser.reset_input_buffer()

    # after which a VS2_START_VS2, 0, 0 (0x16,0x00,0x00) is send
    ser.write([0x16,0x00,0x00])

    # and within 30x100ms an VS2_ACK (0x06) is expected.
    i = 0
    while(i < 30):
        time.sleep(0.1)
        buff = ser.read(1)
        if(len(buff) > 0):
            if(int(buff[0]) == 0x06):
                break
        i+=1

    if(i == 30):
        print("Timeout")
        return False

    return True


def read_data(addr:int, readlen:int, ser:serial.Serial) -> bytes:
    outbuff = bytearray(8)
    outbuff[0] = 0x41   # 0x41 Telegrammstart
    outbuff[1] = 0x05   # Len Nutzdaten, hier immer 5
    outbuff[2] = 0x00   # 0x00 Anfrage
    outbuff[3] = 0x01   # 0x01 Lesen
    outbuff[4] = (addr >> 8) & 0xFF  # hi byte
    outbuff[5] = addr & 0xFF         # lo byte
    outbuff[6] = readlen   # Anzahl der zu lesenden Daten-Bytes
    outbuff[7] = calc_crc(outbuff)

    ser.reset_input_buffer()
    # After message is send, 
    ser.write(outbuff)

    # for up 30x100ms serial data is read.
    i = 0
    state = 0
    inbuff = []
    while(i < 30):
        time.sleep(0.1)
        inbuff += ser.read(ser.in_waiting)

        if(state == 0):
            if(len(inbuff) > 0):
                if(inbuff[0] == 0x06): # VS2_ACK
                    state = 1
                elif (inbuff[0] == 0x15): # VS2_NACK
                    return []
        
        if(state == 1):
            if(len(inbuff) > 2):
                if(inbuff[1] != 0x41): # STX
                    return []
                state = 2

        if(state == 2):
            dlen = inbuff[2]
            if(len(inbuff) >= dlen+4):  # 0x06 + 0x41 + Len + Nutzdaten + CRC
                crc = inbuff[dlen+3] 
                if(crc != calc_crc(inbuff)):
                    print("CRC Error")
                    return []
                return inbuff[8:8+readlen]

        i+=1

    if(i == 30):
        print("Timeout")
        return []


def calc_crc(telegram):
    CRCsum = 0
    telestart = 1
    teleend = telegram[1] + 1

    if telegram[0] != 0x41:
        telestart += 1
        teleend = telegram[2] + 2
        if (telegram[0] != 0x06) and (telegram[1] != 0x41):
            return 0

    for i in range(telestart, teleend + 1):
        CRCsum += telegram[i]

    return CRCsum % 0x100


def bbbstr(data_buffer):
    return ' '.join([format(byte, '02X') for byte in data_buffer])

def bytesval(data, div=1, signd=True):
    return int.from_bytes(data, byteorder='little', signed=signd) / div



# Hauptfunktion
def main():
    port = 'COM4'

    if(len(sys.argv) > 1):
        port = sys.argv[1]

    # Serielle Port-Einstellungen
    ser = serial.Serial(port, baudrate=4800, bytesize=8, parity='E', stopbits=2, timeout=0) 

    try:
        # Serial Port öffnen
        if not ser.is_open:
            ser.open()

        if not init_vs2(ser):
            raise Exception("init_vs2 failed.")

        while(True):
            buff = read_data(0x00f8, 8, ser)
            print("0x00f8", bbbstr(buff))

            buff = read_data(0x0800, 2, ser)
            print("AT", bbbstr(buff), bytesval(buff, 10))

            buff = read_data(0x0804, 2, ser)
            print("WW", bbbstr(buff), bytesval(buff, 10))

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nAufzeichnung beendet.")
    finally:
        # Serial Port schließen
        if ser.is_open:
            print("exit close")
            # re-init KW protocol
            ser.write([0x04])
            ser.close()


main()