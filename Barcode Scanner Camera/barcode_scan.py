import cv2
from pyzbar.pyzbar import decode

# initialize the video capture device
cap = cv2.VideoCapture(0)

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

    # display the frame
    cv2.imshow("Barcode Scanner", frame)

    # exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# release the video capture device and close the window
cap.release()
cv2.destroyAllWindows()
