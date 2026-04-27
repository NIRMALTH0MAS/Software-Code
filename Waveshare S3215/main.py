import time
from python_st3215 import ST3215

# Replace 'COM5' with the actual COM port your driver board is connected to
SERIAL_PORT = 'COM5'
BAUD_RATE = 1000000
SERVO_ID_1 = 1
SERVO_ID_2 = 2

# Configuration
MAX_VALUE = 100
MAX_TURNS = 3
STEPS_PER_TURN = 4096
TOTAL_STEPS = MAX_TURNS * STEPS_PER_TURN # 12288

def map_value_to_steps(value):
    """
    Maps 0-100 to 0-12288 steps (3 rotations).
    """
    # Clamp input between 0 and 100
    if value < 0:
        value = 0
    if value > MAX_VALUE:
        value = MAX_VALUE
        
    # Scale to our total steps
    target_pos = int((value / MAX_VALUE) * TOTAL_STEPS)
    return target_pos

def configure_multi_turn(servo):
    """
    Configure the ST3215 for Step mode (Multi-turn Absolute),
    by setting Mode 3 and removing the angle limits.
    """
    print("Configuring motor for Multi-Turn Mode (Mode 3)...")
    # Need to disable torque before modifying EEPROM values
    servo.sram.torque_disable()
    time.sleep(0.1)
    
    # Mode 3 = Step servo mode (multi-turn)
    current_mode = servo.eeprom.read_operating_mode()
    if current_mode != 3:
        servo.eeprom.write_operating_mode(3)
        time.sleep(0.05)
        
    # Multi-turn relies on the min and max limits being 0
    # Bypassing the python library decorator limits to push raw 0s
    servo._write_memory(0x09, [0, 0]) # Min limit
    time.sleep(0.05)
    servo._write_memory(0x0B, [0, 0]) # Max limit
    time.sleep(0.05)
    
    # Re-enable torque
    servo.sram.torque_enable()
    time.sleep(0.1)

def main():
    print(f"Connecting to motor on {SERIAL_PORT}...")
    
    try:
        motor = ST3215(port=SERIAL_PORT, baudrate=BAUD_RATE)
        servo1 = motor.wrap_servo(SERVO_ID_1, verify=True)
        servo2 = motor.wrap_servo(SERVO_ID_2, verify=True)
        print("Connected!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # Configure hardware for 3 full rotations
    configure_multi_turn(servo1)
    configure_multi_turn(servo2)
    
    # Set a decent speed and acceleration
    servo1.sram.write_acceleration(100)
    servo1.sram.write_running_speed(2500)
    servo2.sram.write_acceleration(100)
    servo2.sram.write_running_speed(2500)
    
    print("\n--- Motor Control Interace ---")
    print("Format to enter: <ServoID> <percentage>")
    print(f"Valid Servo IDs: {SERVO_ID_1}, {SERVO_ID_2}")
    print("Example: '1 50' moves Servo 1 to 50%")
    print("- Increasing the value moves Clockwise (Forward).")
    print("- Decreasing the value moves Anti-Clockwise (Reverse).")
    print("- '100' corresponds exactly to 3 complete rotations from the '0' position.")
    print("- Type 'q' or 'quit' to exit.\n")
    
    # Keep track of where we THINK the motor is (our internal absolute tracker)
    current_target_steps = {SERVO_ID_1: 0, SERVO_ID_2: 0}
    servos = {SERVO_ID_1: servo1, SERVO_ID_2: servo2}
    
    # Set the initial baseline position to wherever it is right now, or force to absolute 0
    print("Zeroing the motors to absolute 0. Stand clear...")
    servo1.sram.write_target_location(0) 
    servo2.sram.write_target_location(0) 
    time.sleep(3) # Give it time to hit exactly 0
    
    while True:
        try:
            user_input = input(f"Enter <ID> <percentage> (e.g. '{SERVO_ID_1} 50') (or 'q' to quit): ").strip()
            
            if user_input.lower() in ['q', 'quit', 'exit']:
                break
                
            parts = user_input.split()
            if len(parts) != 2:
                print("Invalid format. Please enter <ServoID> <percentage>. Example: '1 50'")
                continue
                
            sid = int(parts[0])
            value = float(parts[1])
            
            if sid not in servos:
                print(f"Invalid servo ID. Available IDs: {SERVO_ID_1}, {SERVO_ID_2}.")
                continue
                
            # Map user's 0-100 logic to motor steps absolute coordinate
            new_absolute_target = map_value_to_steps(value)
            
            # In Mode 3 (Stepper Mode), the servo treats the target as a RELATIVE step offset from its current location!
            # So to reach an absolute position, we compute the delta (difference) from where it currently is.
            steps_to_move = new_absolute_target - current_target_steps[sid]
            
            print(f"-> Moving Servo {sid} from {current_target_steps[sid]} to {new_absolute_target} (Sending relative delta: {steps_to_move} steps)...")
            servos[sid].sram.write_target_location(steps_to_move)
            
            # Update our tracker
            current_target_steps[sid] = new_absolute_target
            
        except ValueError:
            print("Please enter valid numbers.")
        except KeyboardInterrupt:
            break

    # Clean up
    print("\nClosing connection...")
    motor.close()

if __name__ == '__main__':
    main()
