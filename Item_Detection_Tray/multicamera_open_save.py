import cv2
import datetime

def capture_images():
    # Get the number of connected cameras
    num_cameras = cv2.getBuildInformation().count("videoio")

    for camera_index in range(num_cameras):
        # Open the camera
        cap = cv2.VideoCapture(camera_index)

        # Check if the camera is opened successfully
        if not cap.isOpened():
            print(f"Failed to open camera {camera_index}")
            continue

        # Read an image from the camera
        ret, frame = cap.read()

        # Check if the image was captured successfully
        if not ret:
            print(f"Failed to capture image from camera {camera_index}")
            cap.release()
            continue

        # Generate the filename using the current date and time
        current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"camera{camera_index+1}_{current_time}.jpeg"

        # Save the image
        cv2.imwrite(filename, frame)

        # Release the camera
        cap.release()

        print(f"Image captured from camera {camera_index} and saved as {filename}")

# Run the capture_images function
capture_images()
