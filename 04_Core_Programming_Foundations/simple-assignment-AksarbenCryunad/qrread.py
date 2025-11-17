import cv2
import sys

cap_dev_id = 0
try:
    cap_dev_id = sys.argv[1]
    cap_dev_id = int(sys.argv[1])
except IndexError:
    print(f"Info: Using default camera 0")
    cap_dev_id = 0
except ValueError as e:
    print(f"Error: {e}")
    exit()

# Initialize the QR code detector
qrd = cv2.QRCodeDetector()

# Open the camera.
cap = cv2.VideoCapture(cap_dev_id)
if not cap.isOpened():
    print("Error: Could not open camera")
    exit()
print("Press 'q' to quit")

while True:
    # Read frame from camera
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read frame")
        break

    # Detect and decode QR code
    data, box, _ = qrd.detectAndDecode(frame)

    # If QR code is detected
    if data:
        print("-"*20)
        print("[QR Code detected]")
        print(f"{data}")

        # Draw bounding box around QR code
        if box is not None:
            box = box[0].astype(int)
            cv2.polylines(frame, [box], True, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow('QR Code Reader', frame)

    # Break loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release everything
cap.release()
cv2.destroyAllWindows()