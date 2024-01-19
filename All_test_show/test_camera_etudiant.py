import socket
import cv2
import time
# from imutils.video import VideoStream

# frameWidth = 720
# frameHeight = 480


# capture = cv2.VideoCapture('http://192.168.0.11:8080/video')
capture = cv2.VideoCapture('http://192.168.137.66:4747/video')
# capture = cv2.VideoCapture(0)
# capture = VideoStream(src=0).start()



# capture.set(3, frameWidth)
# capture.set(4, frameHeight)
# capture.set(10, 20) #set brightness
# capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
# print(capture.get(cv2.CAP_PROP_FPS))


successful = True

start_time = time.time()
counter = 0

# cv2.namedWindow("preview", cv2.WINDOW_NORMAL)
while successful:
    counter += 1
    # print(capture.get(cv2.CAP_PROP_FPS))
    # print(capture.get(cv2.VIDEOWRITER_PROP_FRAMEBYTES))

    successful, image = capture.read()
    # image = capture.read()

    # print(capture.get(cv2.CAP_PROP_FPS))
    # print(capture.get(cv2.VIDEOWRITER_PROP_FRAMEBYTES))

    cv2.imshow("preview", image)

    # END:
    keys = cv2.waitKey(1) & 0xFF
    if keys == ord("q"):
        break
finish_time = time.time()
fps = counter / (finish_time-start_time)
print('Frames per second: ' + str(fps))
capture.release()
# out.release()
cv2.destroyAllWindows()
