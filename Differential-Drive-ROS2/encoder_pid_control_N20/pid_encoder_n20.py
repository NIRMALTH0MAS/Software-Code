from gpiozero import DigitalInputDevice, Robot
import Encoder
from time import sleep

SAMPLETIME = 0.1
KP = 0.00038
KD = 0.00001    #0.01
KI = 0.00001   #0.005

r = Robot((6,13),(20,16)) 
e1 = Encoder.Encoder(23, 18)
e2 = Encoder.Encoder(14, 15)


#start the robot
m1_speed = 1
m2_speed = 1
r.value = (m1_speed, m2_speed)

e1_prev_error = 0
e2_prev_error = 0

e1_sum_error = 0
e2_sum_error = 0

#find a sample rate
while True:

    m1_speed = 1
    m2_speed = 1

    if e1.read() > e2.read():
        e1_error = e1.read() - e2.read()
        e1_prev_error = e1_error
        e1_sum_error += e1_error
        print("Error 1 : {} Prev Error 1 : {} Sum Error 1 : {} ".format(e1_error,e1_prev_error,e1_sum_error))
        m1_speed -= (e1_error * KP) + (e1_prev_error * KD) + (e1_sum_error * KI)
        m1_speed = max(min(1, m1_speed), 0)

    elif e2.read() > e1.read():
        e2_error = e2.read()- e1.read()
        e2_prev_error = e2_error
        e2_sum_error += e2_error
        print("Error 2 : {} Prev Error 2 : {} Sum Error 2 : {} ".format(e2_error,e2_prev_error,e2_sum_error))
        m2_speed -= (e2_error * KP) + (e2_prev_error * KD) + (e2_sum_error * KI)
        m2_speed = max(min(1, m2_speed), 0)

    elif e1.read() == e2.read():
        m1_speed = m2_speed
        print("Error : 0 ")

    print("MotorSpeed :",m1_speed,"  ",m2_speed)
    r.value = (m1_speed, m2_speed)
    print("Encoder 1 : {} Encoder 2 : {} ".format(e1.read(),e2.read()))

    sleep(SAMPLETIME)
    print("-----------------------------------------------------------------------")

#     e1_error = TARGET - e1.read()
#     print("Error 1 : ",e1_error)
#     e2_error = TARGET - e2.read()
#     print("Error 2 : ",e2_error)

#     m1_speed = 0.2
#     m2_speed = 0.2

#     m1_speed = (e1_error * KP) + (e1_prev_error * KD) + (e1_sum_error * KI)
#     m2_speed = (e2_error * KP) + (e1_prev_error * KD) + (e2_sum_error * KI)
#     print("MotorSpeed :",m1_speed,"  ",m2_speed)

#     m1_speed = max(min(1, m1_speed), 0)
#     m2_speed = max(min(1, m2_speed), 0)
#     print("MotorSpeed(MAX MIN) :",m1_speed,"  ",m2_speed)
#     r.value = (m1_speed, m2_speed)

#     print("e1 {} e2 {} diff : {}".format(e1.read(), e2.read(),e1.read()- e2.read()))
#     print("m1 {} m2 {}".format(m1_speed, m2_speed))

#     sleep(SAMPLETIME)

#     e1_prev_error = e1_error
#     e2_prev_error = e2_error

#     e1_sum_error += e1_error
#     e2_sum_error += e2_error

#     TARGET = (e1_error-e2_error)
#     print("Target : {}".format(TARGET))
#     print("-----------------------------------------------------------------------")