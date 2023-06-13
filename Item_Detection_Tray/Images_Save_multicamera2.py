import cv2
import datetime

def open_camera(camera_index):
    cap = cv2.VideoCapture(camera_index)

    # Check if the camera is opened successfully
    if not cap.isOpened():
        print(f"Failed to open camera {camera_index}")
        return

    ret, frame = cap.read()
    
    if not ret:
        print(f"Failed to read from camera {camera_index}")
        cap.release()
        return

    # Generate the filename using the current date and time
    current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"camera{camera_index+1}_{current_time}.jpeg"

    # Save the image
    cv2.imwrite(filename, frame)

    # Release the camera
    cap.release()
    print(f"Image captured from camera {camera_index} and saved as {filename}")
    

if __name__ == '__main__':
    cameras = [3,5,7]  # Camera indices to open
    
    for camera_index in cameras:
        open_camera(camera_index)
