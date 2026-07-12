#!/usr/bin/python3

'''
Created on 2022.05.11

@author: mkrej


'''

import serial 
import sys
import select
import struct
import os

DBG_MEAS_CSV_FSAMPLE = 40e3

#UART_PORT = 'COM4'
UART_PORT = '/dev/ttyUSB1'

BAUD_RATE = 115200
BAUD_RATE = 921600

MODE_READ_BIN = False
MODE_READ_BIN = True

SAVE_CHAN = 12

# LOG_RCV - przełącznik czy drukować dbg info odebrane
LOG_RCV = True
LOG_RCV = False


csv = open("track.csv", "w", 1)
ftcsv = open("track-ft.csv", "w", 1)
raw_meas_csv = open("track-raw-meas.csv", "w", 1)
raw_meas_csv.write("tm; rg; carr\n")

if not MODE_READ_BIN:
    log = open("track.log", "w", 1)
    bin = open("track.bin", "wb", 64)
    obs = open("track.bo", "wb", 64)
    ser = serial.Serial(UART_PORT, BAUD_RATE)
    #ser.flushInput()
    print("waiting on port '{}' ...".format(UART_PORT))

else:
    print("MODE_READ_BIN")
    print("reading .bin ...")
    ser  = open("track.bin", "rb")
    log = open("track-off.log", "w", 1)
    obs = open("track-off.bo", "wb", 64)
    
    
csv.write("idx; chan; IP; QP\n")

bytes = 0

#nie wiem co to za 2 początkowe bajty
#if not MODE_READ_BIN:
#    ser.read(2)
    
    
idx  = 0
ftidx = 0;

if False:
    os.chdir("/home/mk/dyskE/MojePrg/_goGeo/goGeoFirmware/rcv_test0")
    print("EXISTS:", os.path.exists("./sats.bin"))

while True:

    if False and sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        rd = sys.stdin.readline()
        #ser.write(rd.encode())
        print("(->)")
        ser.write(struct.pack("<bhhhh", 1, 6, -180, 90, int(4000.5*8)))

    if os.path.exists("./sats.bin"):
        aug  = open("./sats.bin", "rb")
        while True:
            rd = aug.read(9)
            if len(rd) < 9:
                break
            unp = struct.unpack("<bhhhh", rd)
            if False:
                print("[AUG] PRN: {:2}  azel: [{:<3}  {:3}]  vel: {:9}".format(unp[1], unp[2], unp[3], unp[4] / 8.0))
            # id, prn, az, el, vel
            ser.write(struct.pack("<b4h", 1, *(unp[1:5])))
        os.remove("./sats.bin")

    rec = ser.read(1)
    if rec == b"":
        print("(end of file)")
        break

    if not MODE_READ_BIN and rec == b'~' :
        ln = ser.readline()
        ln = ln.decode('utf_8', errors='ignore')
        #ln = ln.replace("~", "")
        print(ln, end='')
        log.write(ln)

        if "DONE" in ln: 
            break

        continue

    #if MODE_READ_BIN:
    #    rec = bytearray(reversed(rec))

    # https://docs.python.org/3/library/struct.html#format-characters

    id = rec[0]
    
        
    # accu frame
    if id == 0 :
        siz = 6
        rec += ser.read(siz - 1)
        if len(rec) < siz:
            print("end (len(rec) < {})".format(siz))
            break
    
        rd = struct.unpack('<bbhh', rec)
    
        chan = rd[1]
        ip = rd[2] * 32
        qp = rd[3] * 32
        if chan == SAVE_CHAN :
            csv.write("{}; {}; {}; {}\n".format(idx, chan, ip, qp))
            idx += 1

        bytes += siz

    # meas frame
    if id == 1 :   
        siz = 18
        rec += ser.read(siz - 1)
        if len(rec) < siz:
            print("end (len(rec) < {})".format(siz))
            break
        rd = struct.unpack('<bbqq', rec)
        if LOG_RCV:
            msg = "-- meas [{}, {:.6f}, {:.3f}]\n".format(rd[1], rd[2]/ 2**32, rd[3] / 2**10)
            print(msg, end = '')
            log.write(msg);
        if rd[1] == SAVE_CHAN :
            raw_meas_csv.write("{:.6f}; {:.3f}\n".format(rd[2] / 2**32, rd[3] / 2**10))
        bytes += siz
    
    # info frame
    if id == 2 :   
        siz = 4
        rec += ser.read(siz - 1)
        if len(rec) < siz:
            print("end (len(rec) < {})".format(siz))
            break
        rd = struct.unpack('<bbbb', rec)
        msg = "-- info [{}, {}, {}]\n".format(rd[1], rd[2], rd[3])
        print(msg, end = '')
        log.write(msg);
        bytes += siz

    # meas time frame	
    if id == 3 :
        siz = 9
        rec += ser.read(siz - 1)
        if len(rec) < siz:
            print("end (len(rec) < {})".format(siz))
            break
        rd = struct.unpack('<bQ', rec)
        if LOG_RCV:
            msg = "-- ts [{}]\n".format(rd[1])
            print(msg, end = '')
            log.write(msg);
        raw_meas_csv.write("{};".format(rd[1] / DBG_MEAS_CSV_FSAMPLE))
        bytes += siz

    # debug filters frame
    if id == 4 :
        siz = 10
        rec += ser.read(siz - 1)
        if len(rec) < siz:
            print("end (len(rec) < {})".format(siz))
            break
        rd = struct.unpack('<bbll', rec)
        chan = rd[1]
        ftpll = rd[2]
        ftdll = rd[3]
        if chan == SAVE_CHAN :
            ftcsv.write("{}; {}; {}; {}\n".format(ftidx, chan, ftpll, ftdll))
            ftidx += 1
        bytes += siz

    # debug dump time
    if id == 5 :
        #todo tu docelowo 10 bo w tej ramce bardzo brakuje PRN !!!
        siz = 9
        rec += ser.read(siz - 1)
        if len(rec) < siz:
            print("end (len(rec) < {})".format(siz))
            break
        #todo tu koniecznie powinien być jeszcze PRN !!!
        #     bo gdyby robić odtwarzanie pomiarów dla kilku kanałów to się popierniczy
        rd = struct.unpack('<bQ', rec)
        bytes += siz

    if id == 6:
        siz = 28
        rec += ser.read(siz - 1)
         
        if len(rec) < siz:
            break      
        rd = struct.unpack('<BBBqqqB', rec)
        
        chan = rd[1]
        band = rd[2]
        rxtow = rd[3]
        carr = rd[4]
        prange = rd[5]
        prn = rd[6]
        obs_id = 0x10
        if band != 0:
            obs_id = 0x11
        
        binobs = struct.pack('<BBqqqB', obs_id, chan, rxtow, carr, prange, prn)  
        obs.write(binobs)
        
        if LOG_RCV:
            msg = "-- obs: [ID {}; Chan {}; Prn {}; Rx Tow {}; Carr {}; Range {};]\n".format(id, chan, prn, rxtow/2**32, carr/2**10, prange/2**10)
            print(msg, end = '')
            log.write(msg)
            
        bytes += siz

    if id > 6:
        print("unknown frame id ({})".format(id))
        #break

    if False:
        print("({})".format(rec))
        log.write("({})\n".format(rec))

    if not MODE_READ_BIN:
        bin.write(rec)
        
        
    if bytes & 0x3FFF == 0:  
        print("-- received bytes == {}".format(bytes))

    
if not MODE_READ_BIN:    
    bin.close()   
    log.close()
    obs.close()
    
csv.close()
ftcsv.close()     
ser.close()
raw_meas_csv.close()

print("-- received bytes == {}".format(bytes))



