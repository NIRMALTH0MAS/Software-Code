import serial
import time
# from RobotCore import *
# from RobotPlcAPI import Supervisor_GetScannerUserQRInput, sys_launch_process

ScannerPortId = 'COM12'
ScannerPortSpeed = 115200
ScannerPortTimeOut = 2
WRONGVALUE = 0
count = 0
maxtime = 0
error_count = 0
cycle_count = 0
LOOPCOUNT = 2


def scanner():
    global WRONGVALUE, count, maxtime, error_count, cycle_count
    temp = True
    Port = serial.Serial(ScannerPortId, ScannerPortSpeed, timeout=ScannerPortTimeOut)
    print("The connection to the QR scanner is: {}".format(Port.is_open))
    if not Port.is_open:
        return

    while True:
        start = time.time()
        cycle_count += 1
        try:
            if temp:
                print("Scan your QR code..")
                temp = False
            data = Port.readline()
            print("data", data)
            data = data.strip()
            data = data.decode('UTF-8')
            
	
           
        except Exception as e:
            error_count += 1
            print("Error occurred while scanning the QR code with message:".format(e))


# if __name__ == '__main__':
scanner()
# ServerScannerPID = sys_launch_process("RobotServerScanner.py", bpythonscript=True)
# while True:
#     data = Supervisor_GetScannerUserQRInput()
#     if data in ['QP-111101', 'QP-222202', 'QP-333302', 'QP-444402']:
#         print("hi")