import cv2

# Camera parameters
focal_length = 50  # Adjust the focal length value accordingly
camera_height = 100  # Adjust the camera height value accordingly

# Function to measure dimensions of an object
def measure_dimensions(contour):
    # Draw a bounding box around the contour
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    # Calculate object dimensions using the provided formula
    object_width = (w * camera_height) / focal_length
    object_height = (h * camera_height) / focal_length
    
    # Display the measured dimensions on the frame
                                                                                                                                             
    return object_width, object_height

# Function to check if an object violates the edge box
def check_edge_violation(contour, frame_width, frame_height):
    x, y, w, h = cv2.boundingRect(contour)
    
    # Calculate the dimensions of the edge box
    box_width = int(0.8 * frame_width)  # 80% of the frame width
    box_height = int(0.8 * frame_height)  # 80% of the frame height
    
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

# Initialize video capture
cap = cv2.VideoCapture(0)  # Use the appropriate camera index if multiple cameras are connected

while True:
    # Read frame from video capture
    ret, frame = cap.read()
    if not ret:
        break
    
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
            cv2.putText(frame, 'Violation', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    
    # Display the frame
    cv2.imshow('Object Detection', frame)
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
