from collections import deque
import cv2
import numpy as np

def smooth_angle(angle, angle_history, window_size=5):
    angle_history.append(angle)
    if len(angle_history) > window_size:
        angle_history.popleft()
    return round(np.mean(angle_history)) 

def is_circle(contour):
    # Calculate the aspect ratio of the contour's bounding box
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = float(w) / h

    # Define a threshold for circular shape (adjust as needed)
    circularity_threshold = 0.8

    return aspect_ratio >= circularity_threshold

def track_red_ball():
    # Open a video capture object (0 for default camera)
    cap = cv2.VideoCapture(0)
    angle_history = deque(maxlen=5)  # Adjust the window size as needed

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        # Convert the frame to HSV (Hue, Saturation, Value) color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the range of the red color in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Threshold the HSV image to get only the red color
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        # Define the range for the red color in the upper hue range
        lower_red = np.array([160, 100, 100])
        upper_red = np.array([180, 255, 255])

        # Threshold the HSV image to get only the red color in the upper hue range
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        # Combine both masks to cover the full range of red color
        mask = mask1 + mask2

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Get the largest contour (assumed to be the ball)
            largest_contour = max(contours, key=cv2.contourArea)

            # Get the centroid of the largest contour
            M = cv2.moments(largest_contour)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Calculate the angle from the center
            center_x, center_y = frame.shape[1] // 2, frame.shape[0] // 2
            angle = np.degrees(np.arctan2(cy - center_y, cx - center_x))
            # Smooth the angle using a simple moving average
            angle = smooth_angle(angle , angle_history=angle_history)
            # Draw a line from the center to the ball
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
            cv2.line(frame, (center_x, center_y), (cx, cy), (0, 255, 0), 2)

            # Display the angle
            cv2.putText(frame, f"Angle: {angle:.2f} degrees", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('Red Ball Tracking', frame)

        # Break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    track_red_ball()
