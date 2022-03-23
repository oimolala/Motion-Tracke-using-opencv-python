# Motion tracker is using Raspberry Pi 4 and Open CV

"""Importing the library""" 
# lib cv2 will be used to process the image and video
import cv2
# lib RPi.GPIO is used to give inputs and outputs from raspberry pi 4 into servo and camera
import RPi.GPIO as GPIO
# lib numpy is used to process the image which has been converted into its matrix form
import numpy as np
# lib time is used to create a time delay on system
import time

""""""
# nothing function is defined to skip the flag on not-needed to define lib
def nothing(x):
    pass
# this fungtion is used to set the servo angle starting from maximum rotation degree to desired frequency rotation degree
def setServoAngle(servo, angle):
    assert angle >=0 and angle <= 360
    pwm = GPIO.PWM(servo, 50)
    pwm.start(8)
    pwm.ChangeDutyCycle(dutyCycle)
    time.sleep(0.1)
    pwm.stop()

# below is variable used to process the picture/video through camera. flog (0) indicate the camera being used
cap = cv2.VideoCapture(0)

# showing the marking window (image which has been processed).
# HSV color format is being used on this project and green color is used as indicator with hue value of (60, 180)
cv2.namedWindow('marking')
cv2.createTrackbar('H Lower','marking',35,255,nothing)
cv2.createTrackbar('H Higher','marking',255,255,nothing)
cv2.createTrackbar('S Lower','marking',130,255,nothing)
cv2.createTrackbar('S Higher','marking',255,255,nothing)
cv2.createTrackbar('V Lower','marking',139,255,nothing)
cv2.createTrackbar('V Higher','marking',255,255,nothing)


# setting servo pin on Raspberry Pi 4
count = 0
x = 0
y = 0
servo1 = 12
servo2 = 13
setX = 90
setY = 90

# Define pergerakan servo pada Raspberry Pi
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo1, GPIO.OUT)
GPIO.setup(servo2, GPIO.OUT)
setServoAngle(servo1, setY)
setServoAngle(servo2, setX)


# The loop is used to define color which will be detected
while(True):
    # Capture frame-by-frame
    ret, img = cap.read()

    ori2 = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 1)  
    hL = cv2.getTrackbarPos('H Lower','marking')
    hH = cv2.getTrackbarPos('H Higher','marking')
    sL = cv2.getTrackbarPos('S Lower','marking')
    sH = cv2.getTrackbarPos('S Higher','marking')
    vL = cv2.getTrackbarPos('V Lower','marking')
    vH = cv2.getTrackbarPos('V Higher','marking')
    LowerRegion = np.array([hL,sL,vL],np.uint8)
    upperRegion = np.array([hH,sH,vH],np.uint8)
    redObject = cv2.inRange(hsv,LowerRegion,upperRegion)
    kernal = np.ones((1,1),"uint8")
    
    # define the shape area which will be detected
    contours, _ = cv2.findContours(redObject, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    min_x = 9999
    MIN_AREA = 1500 
    
    # loop defining desired shape which will be detected

    for cnt in contours:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])

        if area > MIN_AREA :
            cv2.drawContours(ori2, [approx], 0, (0, 0, 0), 5)
            cv2.circle(ori2, (x, y), 5, (0, 255, 0), -1)
            if 7 <= len(approx) < 20:
                cv2.putText(ori2, "Circle Blue", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))
        
        
    # Determine amount of frame which will be read to direct servo movement
    # In this case, the amount of 5 frame is needed to move the servo
    if count > 5 :
        print("Servo1 " , str(setX), "Servo2 ", str(setY))
        if x < 300:
            print(" " + str(setX))
            setX = setX - 5
            setServoAngle(servo2,setX)
        if x > 340:
            print("" + str(setX))
            setX = setX + 5
            setServoAngle(servo2,setX)
        if y < 160:
            print("masukY1 " + str(setY))
            setY = setY - 5
            setServoAngle(servo1,setY)
        if y > 200:
            print("masukY2" + str(setY))
            setY = setY + 5
            setServoAngle(servo1,setY)

        count = 0
        x = 320
        y = 180
    count += 1
    cv2.imshow("HSVOUT",redObject)
    
    cv2.imshow("ORI",ori2)
    

    if cv2.waitKey(1) & 0xFF == ord('q'):
        GPIO.cleanup()
        cap.release()
        cv2.destroyAllWindows()
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()