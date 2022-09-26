__version__ = '0.1.0'

import serial
import serial.tools.list_ports
import time

VIDPID = '239A:8022'
BAUD = 250000
S_TIMEOUT = 0.2   # Serial command timeout
R_TIMEOUT = 10  # Wait for reponse timeout
DEBUG_MODE=True

class SerialDevice(object):
    def __init__(self):
        self.comport = self.get_com_port()
        self.serial_dev = serial.Serial(self.comport, BAUD, timeout=S_TIMEOUT)

    def get_com_port(self):
        all_ports = list(serial.tools.list_ports.comports())
        for port in all_ports:
            if VIDPID in port[2]:
                return port[0]

    def command(self, data_string):
        print(data_string)
        self.serial_dev.flush()
        self.serial_dev.write((data_string + '\n').encode('ascii'))
        tnow = time.time()
        last_resp = ''
        while time.time() - tnow < R_TIMEOUT:
            if (self.serial_dev.inWaiting() > 0):  
                resp = self.serial_dev.readline().decode('ascii')
                if 'ok' in resp:
                    return last_resp
                else:
                    last_resp = resp
                    if DEBUG_MODE:
                        print('serial response:', last_resp)
