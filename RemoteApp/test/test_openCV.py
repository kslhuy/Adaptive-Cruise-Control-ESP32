import cv2
import numpy as np

# Define the range of red color in HSV
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])

# Example camera parameters (adjust according to your setup)
# focal_length = 1000  # Example focal length in pixels
real_width = 8  # Example real width of the rectangle in some unit (e.g., centimeters)

# Open the camera
cap = cv2.VideoCapture('http://192.168.69.88:4747/video')
desired_width = 720
desired_height = 480

cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)


known_object_width = 80
distance_to_object = 400
image_width_pixels = 720
estimated_focal_length = (known_object_width * distance_to_object) / image_width_pixels*10
print(estimated_focal_length)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Convert frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask to extract red regions
    mask = cv2.inRange(hsv_frame, lower_red, upper_red)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(contour)

        # Filter out small contours
        if area > 100:
            # Calculate the bounding rectangle of the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Calculate the center of the rectangle
            center_x = x + w // 2
            center_y = y + h // 2

            # Calculate the distance from the center of the rectangle
            distance = (estimated_focal_length * real_width) / w  # Simple distance calculation

            # Draw the bounding rectangle and display distance
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f'Distance: {distance:.2f} units', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the result
    cv2.imshow('Real-Time Distance Estimation', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()
