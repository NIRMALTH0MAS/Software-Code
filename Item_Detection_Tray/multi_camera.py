import cv2

def open_camera(camera_index):
    cap = cv2.VideoCapture(camera_index)
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print(f"Failed to read from camera {camera_index}")
            break
        
        cv2.imshow(f"Camera {camera_index}", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    cameras = [0, 7, 3, 9, 5]  # Camera indices to open
    
    for camera_index in cameras:
        open_camera(camera_index)
