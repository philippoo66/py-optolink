import serial
import sys
import time

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Optolink VS2 / 300 Protocol, mainly virtual r/w datapoints
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
        print("init_vs2: Timeout waiting for 0x05")
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
        print("init_vs2: Timeout waiting for 0x06")
        return False

    return True


def read_datapoint(addr:int, rdlen:int, ser:serial.Serial) -> bytes:
    outbuff = bytearray(8)
    outbuff[0] = 0x41   # 0x41 Telegrammstart
    outbuff[1] = 0x05   # Len Payload, hier immer 5
    outbuff[2] = 0x00   # 0x00 Request Message
    outbuff[3] = 0x01   # 0x01 Virtual_READ
    outbuff[4] = (addr >> 8) & 0xFF  # hi byte
    outbuff[5] = addr & 0xFF         # lo byte
    outbuff[6] = rdlen   # Anzahl der zu lesenden Daten-Bytes
    outbuff[7] = calc_crc(outbuff)

    ser.reset_input_buffer()
    # After message is send, 
    ser.write(outbuff)
    print("R tx", bbbstr(outbuff))

    err, data = receive_response(ser)
    return data

def read_energy_testWO1C(wkday:int, ser:serial.Serial) -> bytes:
    addr = 0xb800
    outbuff = bytearray(10)
    outbuff[0] = 0x41   # 0x41 Telegrammstart
    outbuff[1] = 0x07   # Len Payload
    outbuff[2] = 0x00   # 0x00 Anfrage
    outbuff[3] = 0x07   # 0x07 Remote_Proc_Req
    outbuff[4] = (addr >> 8) & 0xFF  # hi byte
    outbuff[5] = addr & 0xFF         # lo byte
    outbuff[6] = 0x02   # Anzahl der Daten-Bytes
    outbuff[7] = 0x02   # procedure#
    outbuff[8] = wkday & 0xFF   # day of week
    outbuff[9] = calc_crc(outbuff)

    ser.reset_input_buffer()
    # After message is send, 
    ser.write(outbuff)
    print("R tx", bbbstr(outbuff))

    # for up 30x100ms serial data is read.
    i = 0
    state = 0
    inbuff = []
    while(True):
        time.sleep(0.1)
        inbuff += ser.read(ser.in_waiting)

        if(state == 0):
            if(len(inbuff) > 0):
                if(inbuff[0] == 0x06): # VS2_ACK
                    state = 1
                elif (inbuff[0] == 0x15): # VS2_NACK
                    print("NACK received")
                    # hier müsste ggf noch ein eventueller Rest des Telegrams abgewartet werden 
                    return []
        
        if(state == 1):
            if(len(inbuff) > 2):
                if(inbuff[1] != 0x41): # STX
                    print("STX Error")
                    return []
                state = 2

        if(state == 2):
            dlen = inbuff[2]
            if(len(inbuff) >= dlen+4):  # 0x06 + 0x41 + Len + Nutzdaten + CRC
                print("R rx", bbbstr(inbuff))
                crc = inbuff[dlen+3] 
                if(crc != calc_crc(inbuff)):
                    print("CRC Error")
                    return []
                if(inbuff[3] & 0x0F == 0x03):
                    print("Error Message")
                    return []
                return inbuff   #[8:8+rdlen]

        i+=1
        if(i == 30):
            print("Timeout")
            return []


def write_datapoint(addr:int, data:bytes, ser:serial.Serial) -> bool:
    wrlen = len(data)
    outbuff = bytearray(wrlen+8)
    outbuff[0] = 0x41   # 0x41 Telegrammstart
    outbuff[1] = 5 + wrlen  # Len Payload
    outbuff[2] = 0x00   # 0x00 Request Message
    outbuff[3] = 0x02   # 0x02 Virtual_WRITE 
    outbuff[4] = (addr >> 8) & 0xFF  # hi byte
    outbuff[5] = addr & 0xFF         # lo byte
    outbuff[6] = wrlen  # Anzahl der zu schreibenden Daten-Bytes
    for i in range(int(wrlen)):
        outbuff[7 + i] = data[i]
    outbuff[7 + wrlen] = calc_crc(outbuff)

    ser.reset_input_buffer()
    ser.write(outbuff)
    print("W tx", bbbstr(outbuff))

    err, data = receive_response(ser)
    return (err == 0x01)


def receive_response(ser:serial.Serial):    # -> int, bytearray: Err_Code, Data
    # Err_Code: 01=success, 03=ErrMsg, 15=NACK, 42=STX_Err, FE=CRC_Err, FF=TimeOut (all hex)
    # receives the V2 response to a Virtual_READ or Virtual_WRITE request
    i = 0
    state = 0
    inbuff = []
    data = []
    # for up 30x100ms serial data is read. (we do 300x10ms)
    while(True):
        time.sleep(0.01)
        inbuff += ser.read(ser.in_waiting)

        if(state == 0):
            if(len(inbuff) > 0):
                if(inbuff[0] == 0x06): # VS2_ACK
                    state = 1
                elif (inbuff[0] == 0x15): # VS2_NACK
                    print("NACK Error")
                    return 0x15, []       # hier müsste ggf noch ein eventueller Rest des Telegrams abgewartet werden 
        
        if(state == 1):
            if(len(inbuff) > 2):  # Ack, STX, Len
                if(inbuff[1] != 0x41): # STX
                    print("STX Error")
                    return 0x41, []    # hier müsste ggf noch ein eventueller Rest des Telegrams abgewartet werden
                state = 2

        if(state == 2):
            pllen = inbuff[2]
            if(len(inbuff) >= pllen+4):  # 0x06 + 0x41 + Len + Payload + CRC
                print("rx", bbbstr(inbuff))
                inbuff = inbuff[0:pllen+4]  # make sure no tailing trash 
                data = inbuff[8:-1]   # 0x06 + 0x41 + Len + FnctCode + MsgId + AddrHi + AddrLo + BlkLen (+ Data) + CRC
                crc = inbuff[pllen+3] 
                if(crc != calc_crc(inbuff)):
                    print("CRC Error")
                    return 0xFE, data
                if(inbuff[3] & 0x0F == 0x03):
                    print("Error Message", bbbstr(data))
                    return 0x03, data
                # success
                return 0x01, data 
        # timout
        i+=1
        if(i > 300):
            print("Timeout")
            return 0xFF, inbuff


def calc_crc(telegram):
    # CRC, a modulo-256 addition of bytes from Block Length and the additional bytes.
    CRCsum = 0
    telestart = 1
    teleend = telegram[1] + 1

    if telegram[0] != 0x41: # STX
        telestart += 1
        teleend = telegram[2] + 2
        if (telegram[0] != 0x06) and (telegram[1] != 0x41):
            print("ugly telegram", bbbstr(telegram))
            return 0  # 1:256 that it fits nevertheless

    for i in range(telestart, teleend + 1):
        CRCsum += telegram[i]

    return CRCsum % 0x100


def bbbstr(data_buffer):
    return ' '.join([format(byte, '02X') for byte in data_buffer])

def bytesval(data, div=1, signd=False):
    return int.from_bytes(data, byteorder='little', signed=signd) / div



# Hauptfunktion
def main():
    port = "COM1"  #'/dev/ttyUSB0' #'COM1'

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
        
        
        if(True):
            for wday in range(1):
                buff = read_energy_testWO1C(wday, ser)
                if(len(buff) > 19):
                    print(f"day {wday}:", bbbstr(buff[12:-1]))
                    print(bytesval(buff[12:14], 10))
                    print(bytesval(buff[14:16], 10))
                    print(bytesval(buff[16:18], 10))
                    print(bytesval(buff[18:20], 10))

        # read test
        while(False):
            buff = read_datapoint(0x00f8, 8, ser)
            print("0x00f8", bbbstr(buff))
            time.sleep(0.1)

            buff = read_datapoint(0x0802, 2, ser)
            print("KT", bbbstr(buff), bytesval(buff, 10))
            time.sleep(0.1)

            buff = read_datapoint(0x0804, 2, ser)
            print("WW", bbbstr(buff), bytesval(buff, 10))
            time.sleep(1)


        # write test
        if(False):
            buff = read_datapoint(0x6300, 1, ser)
            currval = buff
            print("Soll Ist", bbbstr(buff), bytesval(buff))
            
            time.sleep(1)

            data = bytes([50])
            ret = write_datapoint(0x6300, data, ser)
            print("write succ", ret)

            time.sleep(2)

            buff = read_datapoint(0x6300, 1, ser)
            print("Soll neu", bbbstr(buff), bytesval(buff))

            time.sleep(1)

            ret = write_datapoint(0x6300, currval, ser)
            print("write back succ", ret)

            time.sleep(2)

            buff = read_datapoint(0x6300, 1, ser)
            print("Soll read back", bbbstr(buff), bytesval(buff))

    
    except KeyboardInterrupt:
        print("\nGedoehns beendet.")
    except Exception as e:
        print(e)
    finally:
        # Serial Port schließen
        if ser.is_open:
            print("exit close")
            # re-init KW protocol
            ser.write([0x04])
            ser.close()


if __name__ == "__main__":
    main()
