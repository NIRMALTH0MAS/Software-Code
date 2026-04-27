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
time.sleep(2)
\
s.read_all()
\

\
def send_and_expect(cmd, wait_time=2.0):
\
    print(f'>> Sending: {cmd}')
\
    s.write((cmd + '\n').encode('utf-8'))
\
    time.sleep(wait_time)
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
send_and_expect('T 127', 10.0) # Wait 10 seconds for tuning loops
\
s.close()
\

