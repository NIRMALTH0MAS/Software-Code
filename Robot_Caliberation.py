import keyboard

def handle_arrow_key(event, vertical, horizontal, increment):
    if event.name == '+':
        increment += 1
    elif event.name == '-':
        increment -= 1
    elif event.name == 'up':
        vertical += increment
    elif event.name == 'down':
        vertical -= increment
    elif event.name == 'left':
        horizontal -= increment
    elif event.name == 'right':
        horizontal += increment
    return vertical, horizontal, increment

# Example usage1
update_vertical = int(input("Enter the current Vertical Pulse: "))
update_horizontal = int(input("Enter the current Horizontal Pulse: "))
increment = 1

while True:
    event = keyboard.read_event()
    update_vertical, update_horizontal, increment = handle_arrow_key(event, update_vertical, update_horizontal, increment)
    print("Updated Vertical Position:", update_vertical)
    print("Updated Horizontal Position:", update_horizontal)
    print()
