import cv2
import numpy as np

cap = cv2.VideoCapture(0)  # Use 0 for webcam, or change to the device ID of your camera

while True:
    ret, frame = cap.read()

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the red color range in HSV
    lower_red = np.array([170, 120, 50])
    upper_red = np.array([180, 255, 255])

    # Create a mask for the red color
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Apply the mask to the original image
    red_circles = cv2.bitwise_and(frame, frame, mask=mask)

    # Use Hough Circle Transform to detect circles in the red image
    circles = cv2.HoughCircles(red_circles, cv2.HOUGH_GRADIENT, 1.2, 100, minRadius=50, maxRadius=150)

    if circles is not None:
        # Convert the circles into a list of NumPy arrays
        circles = np.array(circles)

        # Iterate through the circles
        for circle in circles:
            # Extract the center and radius from each circle
            x, y, radius = circle[0]

            # Calculate the angle of the circle from the center
            angle = np.arctan2(y - cy, x - cx) * (180 / np.pi)

            # Draw the red circle on the frame
            cv2.circle(frame, (x, y), radius, (0, 0, 255), 2)

            # Draw a line from the center of the red circle to its edge
            cv2.line(frame, (x, y), (x, y + radius * np.cos(angle)), (0, 0, 255), 2)

            # Draw a text box with the angle
            cv2.putText(frame, "Angle: %.2f degrees" % angle, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    cv2.imshow('Red Circle Tracker', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
