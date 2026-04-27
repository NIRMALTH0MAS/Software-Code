import time
from python_st3215 import ST3215

def test():
    motor = ST3215(port='COM5', baudrate=1000000)
    servo = motor.wrap_servo(1, verify=True)
    
    # Disable torque to write EEPROM safely
    servo.sram.torque_disable()
    time.sleep(0.1)
    
    # Enter mode 3 (Stepper Mode = multi-turn)
    print("Setting operating mode to 3 and angle limits to 0...")
    servo.eeprom.write_operating_mode(3)
    time.sleep(0.05)
    
    # Bypassing the library decorator as waveshare requires 0 for multi-turn
    servo._write_memory(0x09, [0, 0]) # min angle limit
    time.sleep(0.05)
    servo._write_memory(0x0B, [0, 0]) # max angle limit
    time.sleep(0.05)
    
    servo.sram.torque_enable()
    time.sleep(0.1)
    
    print("Current position:", servo.sram.read_current_location())
    
    val = 50
    target = int((val / 100.0) * (3 * 4096))
    print(f"Moving to mapped value {val} (target {target})...")
    servo.sram.write_acceleration(50)
    servo.sram.write_running_speed(1000)
    servo.sram.write_target_location(target)
    
    time.sleep(3)
    print("Current position:", servo.sram.read_current_location())
    
    val = 0
    target = int((val / 100.0) * (3 * 4096))
    print(f"Moving to mapped value {val} (target {target})...")
    servo.sram.write_target_location(target)
    
    time.sleep(3)
    print("Current position:", servo.sram.read_current_location())

    motor.close()

if __name__ == '__main__':
    test()
