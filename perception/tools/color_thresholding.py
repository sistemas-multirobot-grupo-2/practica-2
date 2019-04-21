import cv2
import numpy as np

#Callback to attend eachtime a trackbar is used (nothing will happen)
def nothing(x):
    pass

#VideoCapture objetc, to get images from the /dev/video0
cap = cv2.VideoCapture(0) 

#Initialize windows
window_name = 'Green: [20, 80, 0]-[70, 255, 255]'
cv2.namedWindow(window_name)

#Create trackbars (H: 0-180; S: 0-255; V: 0-255)
cv2.createTrackbar('Min. H', window_name, 0, 180, nothing)
cv2.createTrackbar('Min. S', window_name, 0, 255, nothing)
cv2.createTrackbar('Min. V', window_name, 0, 255, nothing)
cv2.createTrackbar('Max. H', window_name, 0, 180, nothing)
cv2.createTrackbar('Max. S', window_name, 0, 255, nothing)
cv2.createTrackbar('Max. V', window_name, 0, 255, nothing)

#Create "switch" to show every contour or just the bigger one
switch_name = '0 : Binary mask \n1 : Final detection'
cv2.createTrackbar(switch_name, window_name, 0, 1, nothing)

#Main loop
ret = True
while(ret == True):
    #Get a new frame
    ret, frame = cap.read() 

    #Check exit causes (including errors)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print ("[*] Shutting everything down...")
        break
    elif (ret == False):
        print ("[!] ERROR! An error has occured while capturing the image")
        break

    #Get trackbar values
    min_h = cv2.getTrackbarPos('Min. H', window_name)
    min_s = cv2.getTrackbarPos('Min. S', window_name)
    min_v = cv2.getTrackbarPos('Min. V', window_name)
    max_h = cv2.getTrackbarPos('Max. H', window_name)
    max_s = cv2.getTrackbarPos('Max. S', window_name)
    max_v = cv2.getTrackbarPos('Max. V', window_name)

    #HSV-based threshold
    green_min = (min_h, min_s, min_v)
    green_max = (max_h, max_s, max_v)

    h, w = frame.shape[:2] #Get frame shape
    frame =  cv2.resize(frame, (int(2*w/3), int(2*h/3))) #Resize frame (to reduce computational cost)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #Converto to HSV
    mask = cv2.inRange(hsv, green_min, green_max) #Create a mask of "things" in the defined range


    #If the user wants to see only the biggest contour
    if cv2.getTrackbarPos(switch_name, window_name) == 1:
        #Find contours
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        #Search the biggest area and point it
        if len(contours) != 0:
            c = max(contours, key = cv2.contourArea)#Find the biggest area
            x,y,w,h = cv2.boundingRect(c) #Generate bounding box
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2) #Draw bounding box

        cv2.imshow(window_name, frame) #Show raw frame with the final detection
    else:
        cv2.imshow(window_name, mask) #Show binary image with the pixels in the configured range

#Release the camera and destroy the window
cap.release()
cv2.destroyAllWindows()

print ('[*] Program ended')
