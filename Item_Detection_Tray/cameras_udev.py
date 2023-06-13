import os
import re
import subprocess

def list_camera_devices():
    devices = subprocess.check_output('v4l2-ctl --list-devices', shell=True).decode('utf-8')
    return devices

def get_camera_details(device):
    details = subprocess.check_output(f'v4l2-ctl -d {device} --all', shell=True).decode('utf-8')
    return details

def generate_udev_rules(cameras):
    rules_content = ""
    for i, camera in enumerate(cameras):
        rules_content += f'SUBSYSTEM=="video4linux", ATTRS{{idVendor}}=="{camera["vendor_id"]}", ATTRS{{idProduct}}=="{camera["product_id"]}", SYMLINK+="camera{i+1}"\n'
    return rules_content

# List camera devices
devices = list_camera_devices()
print("Camera devices:\n")
print(devices)

# Extract device names
device_names = re.findall(r'/dev/video\d+', devices)

# Retrieve details for each camera device
cameras = []
for device in device_names:
    details = get_camera_details(device)
    vendor_id = re.search(r'idVendor\s+=\s+([^\n]+)', details)
    product_id = re.search(r'idProduct\s+=\s+([^\n]+)', details)
    if vendor_id and product_id:
        cameras.append({
            'device': device,
            'vendor_id': vendor_id.group(1),
            'product_id': product_id.group(1)
        })

# Print camera details
for i, camera in enumerate(cameras):
    print(f"\nCamera Device {i+1} Details:\n")
    print(get_camera_details(camera['device']))

# Generate udev rules
udev_rules = generate_udev_rules(cameras)
print("\nGenerated udev rules:\n")
print(udev_rules)

# Write udev rules to a text file
current_directory = os.getcwd()
udev_rules_file = os.path.join(current_directory, 'udev_rules.txt')
with open(udev_rules_file, 'w') as file:
    file.write(udev_rules)

print(f"\nudev rules written to {udev_rules_file}.")
