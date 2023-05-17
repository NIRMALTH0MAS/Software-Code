import cv2
from pyzbar.pyzbar import decode

# initialize the video capture device
cap = cv2.VideoCapture(0)

# set autofocus region of interest (ROI)
# adjust the values according to your requirements and camera capabilities
roi_x = 0.4  # x-coordinate of the top-left corner of the ROI
roi_y = 0.4  # y-coordinate of the top-left corner of the ROI
roi_width = 0.2  # width of the ROI
roi_height = 0.2  # height of the ROI

# calculate the ROI coordinates in the frame
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
roi_start_x = int(frame_width * roi_x)
roi_start_y = int(frame_height * roi_y)
roi_end_x = roi_start_x + int(frame_width * roi_width)
roi_end_y = roi_start_y + int(frame_height * roi_height)

# loop over frames from the video stream
while True:
    # read a frame from the video stream
    ret, frame = cap.read()

    # decode barcodes in the frame
    barcodes = decode(frame)

    # loop over detected barcodes
    for barcode in barcodes:
        # extract the barcode data and type
        barcode_data = barcode.data.decode("utf-8")
        barcode_type = barcode.type

        # print the barcode data and type
        print(f"Found {barcode_type} barcode: {barcode_data}")

        # extract the barcode bounding box coordinates
        x, y, w, h = barcode.rect

        # draw a bounding box around the barcode
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # draw autofocus region rectangle
    cv2.rectangle(frame, (roi_start_x, roi_start_y), (roi_end_x, roi_end_y), (255, 0, 0), 2)

    # display the frame with bounding boxes and autofocus region
    cv2.imshow("Barcode Scanner", frame)

    # exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# release the video capture device and close the window
cap.release()
cv2.destroyAllWindows()
