import socket
import cv2
import numpy as np
import time


######################################################################
width = 640  # WIDTH OF THE IMAGE
height = 480  # HEIGHT OF THE IMAGE
deadZone =100
######################################################################

startCounter =0

# # CONNECT TO TELLO
# me = Tello()
# me.connect()
# me.for_back_velocity = 0
# me.left_right_velocity = 0
# me.up_down_velocity = 0
# me.yaw_velocity = 0
# me.speed = 0



# print(me.get_battery())

# me.streamoff()
# me.streamon()
# ######################## 

frameWidth = width
frameHeight = height
# cap = cv2.VideoCapture(1)
# cap.set(3, frameWidth)
# cap.set(4, frameHeight)
# cap.set(10,200)


global imgContour
global dir
def empty(a):
    pass



def callback_threashold_change(value):
    threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")
    areaMin = cv2.getTrackbarPos("Area", "Parameters")
    
def callback_AccParameters_change(value):
    threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")
    areaMin = cv2.getTrackbarPos("Area", "Parameters")



def callback_HSV_change(value):
    h_min = cv2.getTrackbarPos("HUE Min","HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")
    
    

cv2.namedWindow("HSV")
cv2.resizeWindow("HSV",640,240)
cv2.createTrackbar("HUE Min","HSV",20,179,callback_HSV_change)
cv2.createTrackbar("HUE Max","HSV",40,179,callback_HSV_change)
cv2.createTrackbar("SAT Min","HSV",148,255,callback_HSV_change)
cv2.createTrackbar("SAT Max","HSV",255,255,callback_HSV_change)
cv2.createTrackbar("VALUE Min","HSV",89,255,callback_HSV_change)
cv2.createTrackbar("VALUE Max","HSV",255,255,callback_HSV_change)

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters",640,240)
cv2.createTrackbar("Threshold1","Parameters",166,255,callback_threashold_change)
cv2.createTrackbar("Threshold2","Parameters",171,255,callback_threashold_change)
cv2.createTrackbar("Area","Parameters",1750,30000,callback_threashold_change)


cv2.namedWindow("ACCParameters")
cv2.resizeWindow("ACCParameters",640,240)
# cv2.createTrackbar("Threshold1","Parameters",166,255,callback_AccParameters_change)
# cv2.createTrackbar("Threshold2","Parameters",171,255,callback_AccParameters_change)
# cv2.createTrackbar("Area","Parameters",1750,30000,callback_AccParameters_change)


# Defaut value 
h_min = 0
h_max = 179 
s_min = 194
s_max = 255
v_min = 13
v_max = 255

threshold1 = 109
threshold2 = 255
areaMin = 932

cv2.setTrackbarPos("HUE Min","HSV",h_min)
cv2.setTrackbarPos("HUE Max","HSV",h_max)
cv2.setTrackbarPos("SAT Min","HSV",s_min)
cv2.setTrackbarPos("SAT Max","HSV",s_max)
cv2.setTrackbarPos("VALUE Min","HSV",v_min)
cv2.setTrackbarPos("VALUE Max","HSV",v_max)


cv2.setTrackbarPos("Threshold1","Parameters",threshold1)
cv2.setTrackbarPos("Threshold2","Parameters",threshold2)
cv2.setTrackbarPos("Area","Parameters",areaMin)




# PID Gain 
Kp = 1
Ki = 0.5
Kd = 0.01



class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def update(self, current_value,info, width  ,dt):
        # "info" :  contain center point (x,y)  of the Object decteted
        # width  : of the image camera 
        x_center, y_center = info[0]
        area = info[1]
        
        error = x_center - width // 2

        # error = self.setpoint - current_value
        self.integral += error
        derivative = error - self.prev_error

        output_angle = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output_angle = int(np.clip(output_angle, 70, 120))

        self.prev_error = error
        

        return output_angle
    

    
def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

def getContours(img,imgContour):
    global dir
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > areaMin:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            #print(len(approx))
            x , y , w, h = cv2.boundingRect(approx)
            cx = int(x + (w / 2))  # CENTER X OF THE OBJECT
            cy = int(y + (h / 2))  # CENTER X OF THE OBJECT

            if (cx <int(frameWidth/2)-deadZone):
                cv2.putText(imgContour, " GO LEFT " , (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                cv2.rectangle(imgContour,(0,int(frameHeight/2-deadZone)),(int(frameWidth/2)-deadZone,int(frameHeight/2)+deadZone),(0,0,255),cv2.FILLED)
                dir = 1
            elif (cx > int(frameWidth / 2) + deadZone):
                cv2.putText(imgContour, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                cv2.rectangle(imgContour,(int(frameWidth/2+deadZone),int(frameHeight/2-deadZone)),(frameWidth,int(frameHeight/2)+deadZone),(0,0,255),cv2.FILLED)
                dir = 2
            elif (cy < int(frameHeight / 2) - deadZone):
                cv2.putText(imgContour, " GO UP ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                cv2.rectangle(imgContour,(int(frameWidth/2-deadZone),0),(int(frameWidth/2+deadZone),int(frameHeight/2)-deadZone),(0,0,255),cv2.FILLED)
                dir = 3
            elif (cy > int(frameHeight / 2) + deadZone):
                cv2.putText(imgContour, " GO DOWN ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1,(0, 0, 255), 3)
                cv2.rectangle(imgContour,(int(frameWidth/2-deadZone),int(frameHeight/2)+deadZone),(int(frameWidth/2+deadZone),frameHeight),(0,0,255),cv2.FILLED)
                dir = 4
            else: dir=0

            cv2.line(imgContour, (int(frameWidth/2),int(frameHeight/2)), (cx,cy),(0, 0, 255), 3)
            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)
            cv2.putText(imgContour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7,(0, 255, 0), 2)
            cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,(0, 255, 0), 2)
            cv2.putText(imgContour, " " + str(int(x)) + " " + str(int(y)), (x - 20, y - 45), cv2.FONT_HERSHEY_COMPLEX,0.7,(0, 255, 0), 2)
        else: dir=0

def display(img):
    cv2.line(img,(int(frameWidth/2)-deadZone,0),(int(frameWidth/2)-deadZone,frameHeight),(255,255,0),3)
    cv2.line(img,(int(frameWidth/2)+deadZone,0),(int(frameWidth/2)+deadZone,frameHeight),(255,255,0),3)
    cv2.circle(img,(int(frameWidth/2),int(frameHeight/2)),5,(0,0,255),5)
    cv2.line(img, (0,int(frameHeight / 2) - deadZone), (frameWidth,int(frameHeight / 2) - deadZone), (255, 255, 0), 3)
    cv2.line(img, (0, int(frameHeight / 2) + deadZone), (frameWidth, int(frameHeight / 2) + deadZone), (255, 255, 0), 3)


# cap = cv2.VideoCapture(0)

cap = cv2.VideoCapture('http://192.168.137.66:4747/video')

while True:

    # GET THE IMAGE FROM TELLO

    # frame_read = me.get_frame_read()
    # myFrame = frame.frame
    ret, frame = cap.read()

    img = cv2.resize(frame, (width, height))
    imgContour = img.copy()
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)




    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    mask = cv2.inRange(imgHsv,lower,upper)
    result = cv2.bitwise_and(img,img, mask = mask)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    imgBlur = cv2.GaussianBlur(result, (7, 7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)

    imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
    kernel = np.ones((5, 5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    getContours(imgDil, imgContour)
    display(imgContour)

    # ################# FLIGHT
    # if startCounter == 0:
    #    me.takeoff()
    #    startCounter = 1


#     if dir == 1:
#        me.yaw_velocity = -60
#     elif dir == 2:
#        me.yaw_velocity = 60
#     elif dir == 3:
#        me.up_down_velocity= 60
#     elif dir == 4:
#        me.up_down_velocity= -60
#     else:
#        me.left_right_velocity = 0; me.for_back_velocity = 0;me.up_down_velocity = 0; me.yaw_velocity = 0
#    # SEND VELOCITY VALUES TO TELLO
#     if me.send_rc_control:
#        me.send_rc_control(me.left_right_velocity, me.for_back_velocity, me.up_down_velocity, me.yaw_velocity)
    print(dir)

    stack = stackImages(0.9, ([img, result], [imgDil, imgContour]))
    cv2.imshow('Horizontal Stacking', stack)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        # me.land()
        break

# cap.release()
cv2.destroyAllWindows()