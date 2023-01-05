#!/usr/bin/env python
import rclpy
import RPi.GPIO as GPIO
from gpiozero import Robot
import Encoder
from time import sleep

SAMPLETIME = 1

class MotorDriver(object):

    def __init__(self, wheel_distance=0.098, wheel_diameter=0.066, i_BASE_PWM=0.001, i_MULTIPLIER_STANDARD=0.1, i_MULTIPLIER_PIVOT=1.0, simple_mode = True):
        
        self.BASE_PWM = i_BASE_PWM
        self.MAX_PWM = 1
        
        #Set Motor Pin & Encode Pin
        self.r = Robot((16,20), (6,13)) 
        e1 = Encoder.Encoder(18, 23)
        e2 = Encoder.Encoder(15, 14)

        self.simple_mode = simple_mode

        # Wheel and chasis dimensions
        self._wheel_distance = wheel_distance
        self._wheel_radius = wheel_diameter / 2.0
        self.MULTIPLIER_STANDARD = i_MULTIPLIER_STANDARD
        self.MULTIPLIER_PIVOT = i_MULTIPLIER_PIVOT

    def __del__(self):
        GPIO.cleanup()

    def set_motor(self, m1_speed, m2_speed):
        self.r.value = (m1_speed, m2_speed)
        print("Set Motor Value=["+str(m1_speed)+","+str(m2_speed)+"]")


    def forward(self):
        self.set_motor(1,1)

    def stop(self):
        self.set_motor(0, 0)

    def reverse(self):
        self.set_motor(-1, -1)

    def left(self):
        self.set_motor(1, 0)

    def left_reverse(self):
        self.set_motor(-1, 0)

    def pivot_left(self):
        self.set_motor(-1, 1)

    def right(self):
        self.set_motor(0, 1)

    def right_reverse(self):
        self.set_motor(0,-1)

    def pivot_right(self):
        self.set_motor(1, -1)

    def set_M1M2_speed(self, rpm_speedM1, rpm_speedM2, multiplier):

        self.set_M1_speed(rpm_speedM1, multiplier)
        self.set_M2_speed(rpm_speedM2, multiplier)
        self.set_motor(self.PWM1,self.PWM2)

    def set_M1_speed(self, rpm_speed, multiplier):

        self.PWM1 = min(float((rpm_speed * multiplier) * self.BASE_PWM), self.MAX_PWM)
        #self.p1.ChangeDutyCycle(self.PWM1)
        print("M1="+str(self.PWM1))

    def set_M2_speed(self, rpm_speed, multiplier):

        self.PWM2 = min(float(rpm_speed * multiplier * self.BASE_PWM), self.MAX_PWM)
        #self.p2.ChangeDutyCycle(self.PWM2)
        print("M2="+str(self.PWM2))

    def calculate_body_turn_radius(self, linear_speed, angular_speed):
        if angular_speed != 0.0:
            body_turn_radius = linear_speed / angular_speed
        else:
            # Not turning, infinite turn radius
            body_turn_radius = None
        return body_turn_radius

    def calculate_wheel_turn_radius(self, body_turn_radius, angular_speed, wheel):

        if body_turn_radius is not None:
            """
            if angular_speed > 0.0:
                angular_speed_sign = 1
            elif angular_speed < 0.0:
                angular_speed_sign = -1
            else:
                angular_speed_sign = 0.0
            """
            if wheel == "right":
                wheel_sign = 1
            elif wheel == "left":
                wheel_sign = -1
            else:
                assert False, "Wheel Name not supported, left or right only."

            wheel_turn_radius = body_turn_radius + ( wheel_sign * (self._wheel_distance / 2.0))
        else:
            wheel_turn_radius = None

        return wheel_turn_radius

    def calculate_wheel_rpm(self, linear_speed, angular_speed, wheel_turn_radius):
        """
        Omega_wheel = Linear_Speed_Wheel / Wheel_Radius
        Linear_Speed_Wheel = Omega_Turn_Body * Radius_Turn_Wheel
        --> If there is NO Omega_Turn_Body, Linear_Speed_Wheel = Linear_Speed_Body
        :param angular_speed:
        :param wheel_turn_radius:
        :return:
        """
        if wheel_turn_radius is not None:
            # The robot is turning
            wheel_rpm = (angular_speed * wheel_turn_radius) / self._wheel_radius
        else:
            # Its not turning therefore the wheel speed is the same as the body
            wheel_rpm = linear_speed / self._wheel_radius

        return wheel_rpm

    def set_wheel_movement(self, right_wheel_rpm, left_wheel_rpm):

        print("W1,W2=["+str(right_wheel_rpm)+","+str(left_wheel_rpm)+"]")

        if right_wheel_rpm > 0.0 and left_wheel_rpm > 0.0:
            #print("All forwards")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)

            if self.simple_mode:
                # We make it turn only on one wheel
                if right_wheel_rpm > left_wheel_rpm:
                    print("GO FORWARDS RIGHT")
                    #self.right()
                if right_wheel_rpm < left_wheel_rpm:
                    print("GO FORWARDS LEFT")
                    #self.left()
                if right_wheel_rpm == left_wheel_rpm:
                    print("GO FORWARDS")
                    #self.forward()
            else:
                print("Else GO FORWARDS")
                #self.forward()



        elif right_wheel_rpm > 0.0 and left_wheel_rpm == 0.0:
            #print("Right Wheel forwards, left stop")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)
            self.left()

        elif right_wheel_rpm > 0.0 and left_wheel_rpm < 0.0:
            #print("Right Wheel forwards, left backwards --> Pivot left")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_PIVOT)
            self.pivot_left()
        elif right_wheel_rpm == 0.0 and left_wheel_rpm > 0.0:
            #print("Right stop, left forwards")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)
            self.right()

        elif right_wheel_rpm < 0.0 and left_wheel_rpm > 0.0:
            #print("Right backwards, left forwards --> Pivot right")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_PIVOT)
            self.pivot_right()
        elif right_wheel_rpm < 0.0 and left_wheel_rpm < 0.0:
            #print("All backwards")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)

            if self.simple_mode:
                # We make it turn only on one wheel
                if abs(right_wheel_rpm) > abs(left_wheel_rpm):
                    #print("GO BACKWARDS RIGHT")
                    self.right_reverse()
                if abs(right_wheel_rpm) < abs(left_wheel_rpm):
                    #print("GO BACKWARDS LEFT")
                    self.left_reverse()
                if right_wheel_rpm == left_wheel_rpm:
                    #print("GO BACKWARDS")
                    self.reverse()
            else:
                self.reverse()

        elif right_wheel_rpm == 0.0 and left_wheel_rpm == 0.0:
            #print("Right stop, left stop")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)
            self.stop()
        else:
            assert False, "A case wasn't considered==>"+str(right_wheel_rpm)+","+str(left_wheel_rpm)
            pass

    def set_cmd_vel(self, linear_speed, angular_speed):

        body_turn_radius = self.calculate_body_turn_radius(linear_speed, angular_speed)

        wheel = "right"
        right_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius,
                                                                   angular_speed,
                                                                   wheel)

        wheel = "left"
        left_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius,
                                                                  angular_speed,
                                                                  wheel)

        right_wheel_rpm = self.calculate_wheel_rpm(linear_speed, angular_speed, right_wheel_turn_radius)
        left_wheel_rpm = self.calculate_wheel_rpm(linear_speed, angular_speed, left_wheel_turn_radius)


        self.set_wheel_movement(right_wheel_rpm, left_wheel_rpm)


