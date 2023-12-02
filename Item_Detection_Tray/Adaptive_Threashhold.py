import cv2
import tkinter as tk
from tkinter import filedialog

def draw_rectangles_on_objects(image_path):
    # Load the image
    original_image = cv2.imread(image_path)

    # Convert the image to grayscale
    gray_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)

    # Apply background subtraction to get the foreground mask
    bg_subtractor = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=100, detectShadows=False)
    foreground_mask = bg_subtractor.apply(gray_image)

    # Find contours in the foreground mask
    contours, _ = cv2.findContours(foreground_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw rectangles around the objects
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(original_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # Display the result
    cv2.imshow("Detected Objects", original_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def open_image_and_process():
    # Create a Tkinter root window (invisible)
    root = tk.Tk()
    root.withdraw()

    # Ask the user to select an image file
    file_path = filedialog.askopenfilename()

    if file_path:
        # Process the selected image
        draw_rectangles_on_objects(file_path)

if __name__ == "__main__":
    open_image_and_process()
