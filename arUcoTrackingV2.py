
import numpy as np
import cv2
import cv2.aruco as aruco
import imutils
import time
import serial
import math

cap = cv2.VideoCapture(0)

def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]



def track(matrix_coefficients, distortion_coefficients):
    #declaration of local variables
    t = 0
    t2 = 0
    s = 0
    xHome = 0
    yHome = 0
    xPrev = 0
    yPrev = 0
    greenBlockXY=[[0,0],[0,0]]
    calcGreenBlockXY=[[0,0],[0,0]]
    x_centerPixel=0
    y_centerPixel=0

    #ser.write("1".encode()) #sends 1 to start robot
    while True:
        timer = cv2.getTickCount()
        ret, frame = cap.read()

        #gray filter
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #aruco marker setup
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250) #swapped to 4x4 for better accuracy
        parameters = aruco.DetectorParameters_create() 
        corners, ids, rejected_img_points = aruco.detectMarkers(
            gray, 
            aruco_dict, 
            parameters=parameters, 
            cameraMatrix=matrix_coefficients, 
            distCoeff=distortion_coefficients)

        #colour definintion for green in RGB
        greenLower = (25, 52, 72)#actually green 25,52,72
        greenUpper = (102, 255, 255)

        #greenLower = (108, 93, 63)#pink
        #greenUpper = (225, 255, 205)#pink

        #blue, red, green
        pinkLower = (90, 50, 70)#pink
        pinkUpper = (140, 255, 255)#pink

        #filter setup for HSV
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        #mask applied to find green in stream
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        


        #outline green blobs
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        #mask applied to find pink in stream
        mask2 = cv2.inRange(hsv, pinkLower, pinkUpper)
        mask2 = cv2.erode(mask2, None, iterations=2)
        mask2 = cv2.dilate(mask2, None, iterations=2)

        #outline pink blobs
        cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts2 = imutils.grab_contours(cnts2)
        center2 = None

        

        

        #aruco marker locator
        if np.all(ids is not None):
            for i in range(0, len(ids)):
                #locate aruco marker, draw bounded box and draw axis
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
                (rvec-tvec).any()
                aruco.drawDetectedMarkers(frame, corners)
                aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.02)


                #calculate coordiante of aruco marker
                x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
                y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
                x_centerPixel = x_sum*.25
                y_centerPixel = y_sum*.25

                #initialize home coordinates
                if xHome == 0 and yHome == 0:
                    xHome = x_centerPixel
                    yHome = y_centerPixel

                yBotLine = corners[0][0][3][1] - corners[0][0][2][1]
                xBotLine = corners[0][0][2][0] - corners[0][0][3][0]
                angle = math.atan(yBotLine/xBotLine)  
                angleDeg = angle*(180/math.pi)
                
                #display aruco marker coordinates
                cv2.putText(frame, "X: " + str(int((int(x_centerPixel)-int(xHome))*3.37)) + " mm", (10,75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 204, 30), 2)#flipped x
                cv2.putText(frame, "Y: " + str(int((int(yHome)-int(y_centerPixel))*3.37)) + " mm", (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 204, 30), 2)
                cv2.putText(frame, "Direction: " + str(int(angleDeg)) + " degrees", (10,125), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, "X: " + str((int(x_centerPixel)-int(xHome))) + " pixels", (10,175), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (31, 196, 255), 2)#flipped x
                cv2.putText(frame, "Y: " + str(int(yHome)-int(y_centerPixel)) + " pixels", (10,200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (31, 196, 255), 2)


                if ((x_centerPixel-xHome))<20 and ((x_centerPixel-xHome))>-20 and (yHome-y_centerPixel)<20 and (yHome-y_centerPixel)>-20:#flipped x
                    cv2.putText(frame, "Inital D is Home!", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 123, 255), 2)


#------------------------------------------------------------------------
                #send data every 20 counts
                s+=1
                if s%15 == 0: #was 20
                    ser.write(str((int(x_centerPixel)-int(xHome))).encode() + ",".encode() + str(int(yHome)-int(y_centerPixel)).encode() + ",".encode())#flipped x
                    if ser.inWaiting():
                        msg = ser.readline() #put received msg in "msg"
                        print(msg) #print "msg"
                        
                    


        #locating green blobs and circling them
        for cntr in cnts:
            ((x,y), radius) = cv2.minEnclosingCircle(cntr)
            M = cv2.moments(cntr)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            #draw circle around green blobs and dispaly coordinates
            if radius > 1:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.putText(frame, "X: " + str((int(x)-int(xHome))) + " / Y: " + str(int(yHome)-int(y)), (int(x),int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)#flipped x
                #cv2.circle(frame, center, 3, (0, 0, 255), -1)

                #print coordinates for first 2 blobs
                if t < 2:
                    print("Block " + str(t) + " Location:")
                    print("x: " + str((int(x)-int(xHome))) + " / y: " + str(int(yHome)-int(y)))#flipped x
                    #print(M)
                    ser.write(str((int(x)-int(xHome))).encode() + ",".encode() + str(int(yHome)-int(y)).encode() + ",".encode())#flipped x #------------------------------------------------------------------------
                    greenBlockXY[t][0] = int(x)
                    greenBlockXY[t][1] = int(y)

                    calcGreenBlockXY[t][0] = int(x)-int(xHome)#int(x)
                    calcGreenBlockXY[t][1] = int(yHome)-int(y)#int(y)

                    print(calcGreenBlockXY[t][0])
                    print(calcGreenBlockXY[t][1])
                    #ser.write("Y: ".encode() + str(int(y)).encode())
                    #ser.write(",".encode())
                    t+=1

        


        #locating pink blobs and circling them
        for cntr2 in cnts2:
            ((x2,y2), radius2) = cv2.minEnclosingCircle(cntr2)
            M2 = cv2.moments(cntr2)
            center2 = (int(M2["m10"] / M2["m00"]), int(M2["m01"] / M2["m00"]))

            #draw circle around green blobs and dispaly coordinates
            if radius2 > 1:
                cv2.circle(frame, (int(x2), int(y2)), int(radius2), (0, 255, 0), 2)
                cv2.putText(frame, "X: " + str((int(x2)-int(xHome))) + " / Y: " + str(int(yHome)-int(y2)), (int(x2),int(y2)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)#flipped x
                #cv2.circle(frame, center, 3, (0, 0, 255), -1)

                #print coordinates for first 2 blobs
                if t2 < 2:
                    print("Block " + str(t2+2) + " Location:")
                    print("x: " + str((int(x2)-int(xHome))) + " / y: " + str(int(yHome)-int(y2)))#flipped x
                    #print(M2)
                    ser.write(str((int(x2)-int(xHome))).encode() + ",".encode() + str(int(yHome)-int(y2)).encode() + ",".encode())#flipped x #------------------------------------------------------------------------
                    #ser.write("Y: ".encode() + str(int(y)).encode())
                    #ser.write(",".encode())
                    t2+=1

        #display fps
        fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
        cv2.putText(frame, "FPS: " + str(int(fps)), (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        #print("t: ")
        #print(t)
        #print("x_centerPixel: ")
        #print(x_centerPixel)
        #print("y_centerPixel: ")
        #print(y_centerPixel)

        if t>1:
            thresh = 45

            xPt1 = x_centerPixel
            yPt1 = y_centerPixel

            xPt2 = greenBlockXY[0][0]
            yPt2 = greenBlockXY[0][1]

            xPt3 = greenBlockXY[1][0]
            yPt3 = greenBlockXY[1][1] 

            xPt1Calc = 0
            yPt1Calc = 0

            xPt2Calc = calcGreenBlockXY[0][0]
            yPt2Calc = calcGreenBlockXY[0][1]

            xPt3Calc = calcGreenBlockXY[1][0]
            yPt3Calc = calcGreenBlockXY[1][1] 

            arThresh = np.array([[xPt3Calc-thresh,yPt3Calc],[xPt1Calc+thresh,yPt1Calc]])
            detThresh = np.linalg.det(arThresh)
            
            arObsQ = np.array([[xPt2Calc,yPt2Calc],[xPt3Calc-thresh,yPt3Calc]])
            detObsQ = np.linalg.det(arObsQ)

            arObsR = np.array([[xPt2Calc,yPt2Calc],[xPt1Calc+thresh,yPt1Calc]])
            detObsR = np.linalg.det(arObsR)


            #rightside calculations
            arThresh_Right = np.array([[xPt3Calc,yPt3Calc],[xPt1Calc+thresh,yPt1Calc]])
            detThresh_Right = np.linalg.det(arThresh_Right)

            arObsQ_Right = np.array([[xPt2Calc,yPt2Calc],[xPt3Calc,yPt3Calc]])
            detObsQ_Right = np.linalg.det(arObsQ_Right)

            arObsR_Right = np.array([[xPt2Calc,yPt2Calc],[xPt1Calc+thresh,yPt1Calc]])
            detObsR_Right = np.linalg.det(arObsR_Right)



            #print("detObsQ/detThresh")
            #print(-(detObsQ/detThresh))
            #print("Done Calcs")
            #print("N1")
            #print(detObsQ)
            #print((xPt2Calc*yPt3Calc)-((xPt3Calc-thresh)*yPt2Calc))
            #print((detObsQ/detThresh))
            #print("M2")
            #print(detObsR)
            #print((xPt2Calc*yPt1Calc)-((xPt1Calc+thresh)*yPt2Calc))
            #print((detObsR/detThresh))
            #print("Block Inside")
            #print(str(xPt3Calc) + " / " + str(yPt3Calc))
            #print("Block Outside")
            #print(str(xPt2Calc) + " / " + str(yPt2Calc))


            #cv2.line(frame, (int(xPt1), int(yPt1)), (int(xPt3),int(yPt3)), (0,255,0),2)
            #slope = (yPt2-yPt1)/(xPt2-xPt1)
            if 0<= -(detObsQ/detThresh) and -(detObsQ/detThresh) <=1 and 0<= (detObsR/detThresh) and (detObsR/detThresh) <=1:
                
                
                 #if block is in threshhold
                cv2.line(frame, (int(xPt1+thresh), int(yPt1)), (int(xPt3+thresh),int(yPt3)), (0,0,255),2)#boundaries 45
                cv2.line(frame, (int(xPt1-thresh), int(yPt1)), (int(xPt3-thresh),int(yPt3)), (0,0,255),2)#boundaries 45
                #print("Block is in threshold")
                if 0<= -(detObsQ_Right/detThresh_Right) and -(detObsQ_Right/detThresh_Right) <=1 and 0<= (detObsR_Right/detThresh_Right) and (detObsR_Right/detThresh_Right) <=1:
                    
                    cv2.line(frame, (int(xPt1), int(yPt1)), (int(xPt2-thresh),int(yPt2)), (255,205,0),2)
                    cv2.line(frame, (int(xPt2-thresh),int(yPt2)), (int(xPt3),int(yPt3)), (255,205,0),2)
                    
                else:
                    
                    #if block is not in threshold
                    cv2.line(frame, (int(xPt1), int(yPt1)), (int(xPt3),int(yPt3)), (0,255,0),2)



            else:
               #if block is not in threshhold
                cv2.line(frame, (int(xPt1), int(yPt1)), (int(xPt3),int(yPt3)), (0,255,0),2)
                cv2.line(frame, (int(xPt1+thresh), int(yPt1)), (int(xPt3+thresh),int(yPt3)), (255,205,0),2)#boundaries 45
                cv2.line(frame, (int(xPt1-thresh), int(yPt1)), (int(xPt3-thresh),int(yPt3)), (255,205,0),2)#boundaries 45
                #print("Block outside threshold")
                #cv2.line(frame, (int(xPt1-75), int(yPt1)), (int(xPt2-75),int(yPt2)), (0,0,255),2)#boundaries 45

        

        #show tracking window
        cv2.imshow("Tracking", frame)

        #press space to start robot
        if cv2.waitKey(1) & 0xff == ord(' '): 
            print("go")
            ser.write("s".encode() + ",".encode())

        cv2.waitKey(1)
        #cv2.destroyAllWindows() 

#import camera calibration
conditions = load_coefficients("camera.yml")
#connect to bluetooth on robot
ser = serial.Serial("COM10", 9600, timeout=2) #------------------------------------------------------------------------
#begin tracking function
track(conditions[0], conditions[1])