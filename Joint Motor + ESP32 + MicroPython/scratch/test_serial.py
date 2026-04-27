\
import serial
\
import time
\
import sys
\

\
try:
\
    s = serial.Serial('COM6', 115200, timeout=1)
\
except Exception as e:
\
    print('Failed to open COM6:', e)
\
    sys.exit(1)
\

\
print('Waiting for ESP32 boot...')
\
time.sleep(2)
\
# flush buffer
\
s.read_all()
\

\
def send_and_expect(cmd):
\
    print(f'>> Sending: {cmd}')
\
    s.write((cmd + '\n').encode('utf-8'))
\
    time.sleep(1.5)
\
    out = s.read(s.in_waiting).decode('utf-8', errors='ignore')
\
    print(out.strip())
\
    print('-'*40)
\

\
send_and_expect('S')
\
send_and_expect('R 127')
\
send_and_expect('P 127 10')
\
send_and_expect('A 127 90')
\
s.close()
\
print('Test sequence complete')
\

