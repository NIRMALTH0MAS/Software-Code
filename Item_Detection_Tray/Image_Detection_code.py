import cv2
import tkinter as tk
from tkinter import filedialog

# Camera parameters
focal_length = 50  # Adjust the focal length value accordingly
camera_height = 100  # Adjust the camera height value accordingly

# Function to check and resize image if necessary
def check_and_resize_image(image):
    height, width, _ = image.shape
    if height > 1080:
        scale_factor = 1080 / height
        resized_image = cv2.resize(image, (int(width * scale_factor), 1080))
        return resized_image
    else :
        print("Image dimesion is in the desired limit")
    return image

# Function to measure dimensions of an object
def measure_dimensions(contour):
    # Draw a bounding box around the contour
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    # Calculate object dimensions using the provided formula
    object_width = (w * camera_height) / focal_length
    object_height = (h * camera_height) / focal_length

    # Display the measured dimensions on the frame
    # cv2.putText(frame, f'Width: {object_width:.2f} mm', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 1)
    # cv2.putText(frame, f'Height: {object_height:.2f} mm', (x, y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 1)
    
    # Display the measured dimensions on the frame
    return object_width, object_height

# Function to check if an object violates the edge box
def check_edge_violation(contour, frame_width, frame_height):
    x, y, w, h = cv2.boundingRect(contour)
    
    # Calculate the dimensions of the edge box
    box_width = int(0.9 * frame_width)  # 80% of the frame width
    box_height = int(0.9 * frame_height)  # 80% of the frame height
    
    # Calculate the boundaries of the edge box
    box_x = int((frame_width - box_width) / 2)  # Center the edge box horizontally
    box_y = int((frame_height - box_height) / 2)  # Center the edge box vertically
    box_right = box_x + box_width
    box_bottom = box_y + box_height
    
    # Calculate the dimensions of the contour
    contour_right = x + w
    contour_bottom = y + h
    
    # Check if the contour violates the edge box
    if x < box_x or contour_right > box_right or y < box_y or contour_bottom > box_bottom:
        # Draw a red rectangle around the contour to indicate violation
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        return True
    
    return False

# Function to draw yellow lines on the image
def draw_yellow_lines(image):
    height, width, _ = image.shape
    left_x = int(width * 0.03)
    right_x = int(width * 0.97)
    bottom_y = int(height * 0.97)
    
    cv2.line(image, (left_x, 0), (left_x, height), (0, 255, 255), 2)
    cv2.line(image, (right_x, 0), (right_x, height), (0, 255, 255), 2)
    cv2.line(image, (0, bottom_y), (width, bottom_y), (0, 255, 255), 2)

# Create tkinter root window
root = tk.Tk()
root.withdraw()

# Open image file using a dialog box
file_path = filedialog.askopenfilename()

# Read image file
image = cv2.imread(file_path)

# Check and resize image if necessary
frame = check_and_resize_image(image)

# Get frame dimensions
frame_height, frame_width, _ = frame.shape

# Convert frame to grayscale
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Apply thresholding to obtain binary image
_, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# Find contours in the binary image
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    # Calculate the dimensions of the contour
    object_width, object_height = measure_dimensions(contour)
    
    # Check if the contour violates the edge box
    if check_edge_violation(contour, frame_width, frame_height):
        cv2.putText(frame, 'Edge Violation', (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        # Draw yellow lines on the image
        draw_yellow_lines(frame)
    else :
        # cv2.putText(frame, 'No Edge Violation', (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        draw_yellow_lines(frame)

# Display the frame
cv2.imshow('Object Detection', frame)
cv2.waitKey(0)

# Close OpenCV windows
cv2.destroyAllWindows()
