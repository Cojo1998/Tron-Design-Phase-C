
import numpy as np
import cv2
import cv2.aruco as aruco
import imutils
import time
import serial
import math
import shapefile
from shapely.geometry import Point, Polygon

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
    c = 0
    greenBlockXY=[[0,0],[0,0]]
    calcGreenBlockXY=[[0,0],[0,0]]
    blueBlockXY=[[0,0],[0,0]]
    calcBlueBlockXY=[[0,0],[0,0]]
    x_centerPixel=0
    y_centerPixel=0
    calculatePath=1

    def checkPath(x1, y1, x2, y2, x3, y3, x4, y4, xDropG, yDropG, xDropB, yDropB, xHome, yHome):
        arCoord = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)] #coordinates of blocks
        thresh = 60 #set threshold for path

        d = 0
        i = 0
        m = 0
        n = 0
        k = 0
        j = 0
        p = 0
        pathClear = 0

        #begin path finding algorithm
        while i < 4:
            print("Enter while i")
            print("d: " + str(d))
            k = 0
            #make threshold polygon coordinates for inital block
            if d == 0: 
                obstacleCheck = [(arCoord[i][0]-thresh, arCoord[i][1]), (arCoord[i][0]+thresh, arCoord[i][1]), (xHome+thresh, yHome), (xHome-thresh, yHome)]
                print(obstacleCheck)

            #make threshold polygon from block to dropoff   
            if d == 1:
                #if green do green drop off
                if i < 2: 
                    obstacleCheck = [(arCoord[i][0]-thresh, arCoord[i][1]), (arCoord[i][0]+thresh, arCoord[i][1]), (xDropG+thresh, yDropG), (xDropG-thresh, yDropG)]
                #else do blue drop off
                else: 
                    obstacleCheck = [(arCoord[i][0]-thresh, arCoord[i][1]), (arCoord[i][0]+thresh, arCoord[i][1]), (xDropB+thresh, yDropB), (xDropB-thresh, yDropB)]

            #first path is clear, check drop off to next block      
            if d == 2:
                #to avoid checking missing blocks
                if m == i: 
                    m+=1
                #prevents from going outside of array
                if m > 3: 
                    m = 3
                #if in green drop off
                if i < 2: 
                    obstacleCheck = [(xDropG+thresh, yDropG), (xDropG-thresh, yDropG), (arCoord[m][0]-thresh, arCoord[m][1]), (arCoord[m][0]+thresh, arCoord[m][1])]
                #else do blue drop off
                else: 
                    obstacleCheck = [(xDropB+thresh, yDropB), (xDropB-thresh, yDropB), (arCoord[m][0]-thresh, arCoord[m][1]), (arCoord[m][0]+thresh, arCoord[m][1])]

            #check path to drop off
            if d == 3: 
                #if green
                if m < 2: 
                    obstacleCheck = [(arCoord[m][0]-thresh, arCoord[m][1]), (arCoord[m][0]+thresh, arCoord[m][1]), (xDropG+thresh, yDropG), (xDropG-thresh, yDropG)]
                #if blue
                else: 
                    obstacleCheck = [(arCoord[m][0]-thresh, arCoord[m][1]), (arCoord[m][0]+thresh, arCoord[m][1]), (xDropB+thresh, yDropB), (xDropB-thresh, yDropB)]

            #choose next block
            if d == 4:
                #incrememnt n until its not a value of a missing block 
                while n == i or n== m: 
                    n+=1
                #prevents from going outside array
                if n > 3: 
                    n = 3
                #if currently in green drop off
                if n < 2: 
                    obstacleCheck = [(xDropG+thresh, yDropG), (xDropG-thresh, yDropG), (arCoord[n][0]-thresh, arCoord[n][1]), (arCoord[n][0]+thresh, arCoord[n][1])]
                #else do blue drop off
                else: 
                    obstacleCheck = [(xDropB+thresh, yDropB), (xDropB-thresh, yDropB), (arCoord[n][0]-thresh, arCoord[n][1]), (arCoord[n][0]+thresh, arCoord[n][1])]

            #check if path clear to drop off
            if d == 5: 
                #if green
                if n > 2: 
                    obstacleCheck = [(arCoord[n][0]-thresh, arCoord[n][1]), (arCoord[n][0]+thresh, arCoord[n][1]), (xDropG+thresh, yDropG), (xDropG-thresh, yDropG)]
                #if blue
                else:
                    obstacleCheck = [(arCoord[n][0]-thresh, arCoord[n][1]), (arCoord[n][0]+thresh, arCoord[n][1]), (xDropB+thresh, yDropB), (xDropB-thresh, yDropB)]

            #found clear path
            if d == 6:
                pathClear = 1

            #if still looking for path
            if pathClear == 0:
                print("enter clear path")
                #if value equals same coordinate restart loop
                if (m == i or n == m or n == i) and m > 0 and n > 0 and i > 0: 
                    print("nothing works")
                    k = 0
                else:
                    poly = Polygon(obstacleCheck) #set up threshold as polygon

                    #check all coordinates inside threshold
                    while j < 4: 
                        print("Enter while j")
                        #if 1 block dropped off already
                        if d == 2 or d == 3: 
                            #dont check missing block
                            if j == i: 
                                j+=1
                                k+=1
                            else:
                                pt = Point(arCoord[j][0], arCoord[j][1])
                                #if block isnt in threshold
                                if pt.within(poly) == False: 
                                    k+=1
                                j+=1
                        #checking when 2 blocks dropped off
                        if d == 4 or d == 5 : 
                            #dont check missing blocks
                            if j == i or j == m: 
                                j+=1
                                k+=1
                            else:
                                pt = Point(arCoord[j][0], arCoord[j][1])
                                #if block isnt in threshold
                                if pt.within(poly) == False: 
                                    k+=1
                                j+=1
                        if d < 2:
                            pt = Point(arCoord[j][0], arCoord[j][1])
                            print(pt)
                            #if block isnt in threshold
                            if pt.within(poly) == False:
                                print(pt.within(poly)) 
                                k+=1
                            j+=1
                #if path clear go to next stage
                print("i: " + str(i))
                print("k: " + str(k))
                j = 0
                if k > 3: 
                    d+=1
                else:
                    #if on stage 2
                    if d == 2: 
                        #if went through all leftover blocks
                        if m > 3: 
                            i+=1
                        #if havent gone through all blocks
                        else: 
                            m+=1
                            #dont check itself
                            if m == i: 
                                m+=1
                    #choose different block, go back a stage
                    if d == 3: 
                        m+=1 #choose a different block
                        d = 2 #go back to block choosing stage
                    #choose different block
                    if d == 4:
                        #if went through all leftover blocks
                        if n > 3:
                            m+=1 #choose a different block
                            d = 2 #go back to previous block choosing stage
                        else:
                            n+=1
                    #choose different block
                    if d == 5:
                        n+=1 #choose different block
                        d = 4 #go back to block picking stage
                    if d < 2 or m > 3: #if still planning first path, or all paths blocked for stage 2
                        i+=1
            else:
                #finds last coordinate
                while p == i or p == m or p == n: 
                    p+=1
                    if p > 3:
                        p = 0
                    #0 is green, 1 is blue
                    if i < 2: #check colour of block
                        colour1 = 1
                    else:
                        colour1 = 2
                    if m < 2: #check colour of block
                        colour2 = 1
                    else:
                        colour2 = 2
                    if n < 2: #check colour of block
                        colour3 = 1
                    else:
                        colour3 = 2
                    if p < 2: #check colour of block
                        colour4 = 1
                    else:
                        colour4 = 2
                #path of robot
                arPath = [(colour1, arCoord[i][0], arCoord[i][1]), (colour2, arCoord[m][0], arCoord[m][1]), (colour3, arCoord[n][0], arCoord[n][1]), (colour4, arCoord[p][0], arCoord[p][1]), (0, 0, i), (0, 0, m), (0, 0, n), (0, 0, p)]
                i = 8 #ends while loop

        #if path found
        if i == 8:
            print("FOUND CLEAR PATH")
            return arPath #return path
        #if all fails, go in regular order
        else: 
            arPath = [(0, arCoord[0][0], arCoord[0][1]), (0, arCoord[1][0], arCoord[1][1]), (1, arCoord[2][0], arCoord[2][1]), (1, arCoord[3][0], arCoord[3][1]), (0, 0, 0), (0, 0, 1), (0, 0, 2), (0, 0, 3)]
            print("NO PATH FOUND")
            return arPath






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
        greenLower = (25, 45, 80)#actually green 25,52,72
        greenUpper = (90, 255, 255)

        #greenLower = (108, 93, 63)#pink
        #greenUpper = (225, 255, 205)#pink

        #blue, red, green
        pinkLower = (90,50,70)#pink 90,38,80 (old 90,50,70) (new 90,35,105)
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
                cv2.putText(frame, "X: " + str(int((int(x_centerPixel)-int(xHome))*-3.37)) + " mm", (10,75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 204, 30), 2)#flipped x
                cv2.putText(frame, "Y: " + str(int((int(yHome)-int(y_centerPixel))*3.37)) + " mm", (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 204, 30), 2)
                cv2.putText(frame, "Direction: " + str(int(angleDeg)) + " degrees", (10,125), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, "X: " + str(-(int(x_centerPixel)-int(xHome))) + " pixels", (10,175), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (31, 196, 255), 2)#flipped x
                cv2.putText(frame, "Y: " + str(int(yHome)-int(y_centerPixel)) + " pixels", (10,200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (31, 196, 255), 2)


                if (-(x_centerPixel-xHome))<20 and (-(x_centerPixel-xHome))>-20 and (yHome-y_centerPixel)<20 and (yHome-y_centerPixel)>-20:#flipped x
                    cv2.putText(frame, "Inital D is Home!", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 123, 255), 2)


#------------------------------------------------------------------------
                #send data every 20 counts
                
                
                s+=1
                if s%10 == 0: #was 20
                    ser.write(str(-(int(x_centerPixel)-int(xHome))).encode() + ",".encode() + str(int(yHome)-int(y_centerPixel)).encode() + ",".encode())#flipped x
                    if ser.inWaiting():
                        msg = ser.readline() #put received msg in "msg"
                        print(msg) #print "msg"
                        
                    


        #locating green blobs and circling them
        for cntr in cnts:
            ((x,y), radius) = cv2.minEnclosingCircle(cntr)
            M = cv2.moments(cntr)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            #draw circle around green blobs and dispaly coordinates
            if radius > 3:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.putText(frame, "X: " + str(-(int(x)-int(xHome))) + " / Y: " + str(int(yHome)-int(y)), (int(x),int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)#flipped x
                #cv2.circle(frame, center, 3, (0, 0, 255), -1)

                #print coordinates for first 2 blobs
                if t < 2:
                    print("Block " + str(t) + " Location:")
                    print("x: " + str(-(int(x)-int(xHome))) + " / y: " + str(int(yHome)-int(y)))#flipped x
                    #print(M)
                    #ser.write(str(-(int(x)-int(xHome))).encode() + ",".encode() + str(int(yHome)-int(y)).encode() + ",".encode())#flipped x #------------------------------------------------------------------------
                    
                    greenBlockXY[t][0] = int(x)
                    greenBlockXY[t][1] = int(y)

                    calcGreenBlockXY[t][0] = -(int(x)-int(xHome))#flipped x
                    calcGreenBlockXY[t][1] = int(yHome)-int(y)#int(y)

                    #print(calcGreenBlockXY[t][0])
                    #print(calcGreenBlockXY[t][1])
                    #ser.write("Y: ".encode() + str(int(y)).encode())
                    #ser.write(",".encode())
                    t+=1

        


        #locating pink blobs and circling them
        for cntr2 in cnts2:
            ((x2,y2), radius2) = cv2.minEnclosingCircle(cntr2)
            M2 = cv2.moments(cntr2)
            center2 = (int(M2["m10"] / M2["m00"]), int(M2["m01"] / M2["m00"]))

            #draw circle around green blobs and dispaly coordinates
            if radius2 > 3 and radius2 < 6:
                cv2.circle(frame, (int(x2), int(y2)), int(radius2), (0, 255, 0), 2)
                cv2.putText(frame, "X: " + str(-(int(x2)-int(xHome))) + " / Y: " + str(int(yHome)-int(y2)), (int(x2),int(y2)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)#flipped x
                #cv2.circle(frame, center, 3, (0, 0, 255), -1)

                #print coordinates for first 2 blobs
                if t2 < 2:
                    if (-(int(x2)-int(xHome))) > 15 or (int(yHome)-int(y2)) > 15:
                        print("Block " + str(t2+2) + " Location:")
                        print("x: " + str(-(int(x2)-int(xHome))) + " / y: " + str(int(yHome)-int(y2)))#flipped x
                        #print(M2)
                        #ser.write(str(-(int(x2)-int(xHome))).encode() + ",".encode() + str(int(yHome)-int(y2)).encode() + ",".encode())#flipped x #------------------------------------------------------------------------
                        
                        blueBlockXY[t2][0] = int(x2)#flipped x
                        blueBlockXY[t2][1] = int(y2)

                        calcBlueBlockXY[t2][0] = -(int(x2)-int(xHome))#flipped x
                        calcBlueBlockXY[t2][1] = int(yHome)-int(y2)#int(y)

                        #ser.write("Y: ".encode() + str(int(y)).encode())
                        #ser.write(",".encode())
                        t2+=1

        #display fps
        fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
        cv2.putText(frame, "FPS: " + str(int(fps)), (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if t>1 and t2>1: #>1

            xDropG = -7
            yDropG = 406

            xDropB = 531
            yDropB = 404

            if calculatePath == 1:
                print("Calculating best route...")
                bestPath = checkPath(calcGreenBlockXY[0][0], calcGreenBlockXY[0][1], calcGreenBlockXY[1][0], calcGreenBlockXY[1][1], calcBlueBlockXY[0][0], calcBlueBlockXY[0][1], calcBlueBlockXY[1][0], calcBlueBlockXY[1][1], xDropG, yDropG, xDropB, yDropB, 0, 0)
                #bestPath = checkPath(greenBlockXY[0][0], greenBlockXY[0][1], greenBlockXY[1][0], greenBlockXY[1][1], blueBlockXY[0][0], blueBlockXY[0][1], blueBlockXY[1][0], blueBlockXY[1][1], xDropG, yDropG, xDropB, yDropB, xHome, yHome)
                print("Recommended Route:")
                #print(bestPath)
                arLineDraw = [(greenBlockXY[0][0], greenBlockXY[0][1]), (greenBlockXY[1][0], greenBlockXY[1][1]), (blueBlockXY[0][0], blueBlockXY[0][1]), (blueBlockXY[1][0], blueBlockXY[1][1])]
                print("Block 1:")
                print("x: " + str(int(bestPath[0][1])) + " / y: " + str(int(bestPath[0][2])))
                print("Block 2:")
                print("x: " + str(int(bestPath[1][1])) + " / y: " + str(int(bestPath[1][2])))
                print("Block 3:")
                print("x: " + str(int(bestPath[2][1])) + " / y: " + str(int(bestPath[2][2])))
                print("Block 4:")
                print("x: " + str(int(bestPath[3][1])) + " / y: " + str(int(bestPath[3][2])))
                w = 0
                while w < 4:
                    ser.write(str(int(bestPath[w][0])).encode() + ",".encode() + str(int(bestPath[w][1])).encode() + ",".encode() + str(int(bestPath[w][2])).encode() + ",".encode())
                    print("sent block " + str(w))
                    w+=1
                calculatePath = 0

            xPt1 = x_centerPixel
            yPt1 = y_centerPixel

            arLineDraw = [(greenBlockXY[0][0], greenBlockXY[0][1]), (greenBlockXY[1][0], greenBlockXY[1][1]), (blueBlockXY[0][0], blueBlockXY[0][1]), (blueBlockXY[1][0], blueBlockXY[1][1])]

            xPt2 = arLineDraw[bestPath[4][2]][0]
            yPt2 = arLineDraw[bestPath[4][2]][1]

            xPt3 = arLineDraw[bestPath[5][2]][0]
            yPt3 = arLineDraw[bestPath[5][2]][1]

            xPt4 = arLineDraw[bestPath[6][2]][0]
            yPt4 = arLineDraw[bestPath[6][2]][1]

            xPt5 = arLineDraw[bestPath[7][2]][0]
            yPt5 = arLineDraw[bestPath[7][2]][1]

            xDropG = 565
            yDropG = 52

            xDropB = 50
            yDropB = 50

            if c < 1:
                cv2.line(frame, (int(xPt1), int(yPt1)), (int(xPt2),int(yPt2)), (0,255,0),2)
                #cv2.line(frame, (int(xPt1-55), int(yPt1)), (int(xPt2-55),int(yPt2)), (0,255,0),2)
                #cv2.line(frame, (int(xPt1+55), int(yPt1)), (int(xPt2+55),int(yPt2)), (0,255,0),2)
                if xPt1 < xPt2+15 and xPt1 > xPt2-15 and yPt1 < yPt2+15 and yPt1 > yPt2-15:
                    c+=1
            if c < 2:
                if bestPath[0][0] == 1:
                    if c == 1:
                        cv2.line(frame, (int(xPt1), int(yPt1)), (int(xDropG),int(yDropG)), (0,255,0),2)
                    else:
                        cv2.line(frame, (int(xPt2), int(yPt2)), (int(xDropG),int(yDropG)), (0,255,0),2)
                    if xPt1 < xDropG+15 and xPt1 > xDropG-15 and yPt1 < yDropG+15 and yPt1 > yDropG-15 and c > 0:
                        c+=1
                else:
                    if c == 1:
                        cv2.line(frame, (int(xPt1), int(yPt1)), (int(xDropB),int(yDropB)), (0,255,0),2)
                    else:
                        cv2.line(frame, (int(xPt2), int(yPt2)), (int(xDropB),int(yDropB)), (0,255,0),2)
                    if xPt1 < xDropB+15 and xPt1 > xDropB-15 and yPt1 < yDropB+15 and yPt1 > yDropB-15  and c > 0:
                        c+=1
                
            if c < 3:
                if bestPath[0][0] == 1:
                    if c == 2:
                        cv2.line(frame, (int(xPt1),int(yPt1)), (int(xPt3),int(yPt3)), (255,255,0),2)
                    else:
                        cv2.line(frame, (int(xDropG),int(yDropG)), (int(xPt3),int(yPt3)), (255,255,0),2)
                else:
                    if c == 2:
                        cv2.line(frame, (int(xPt1),int(yPt1)), (int(xPt3),int(yPt3)), (255,255,0),2)
                    else:
                        cv2.line(frame, (int(xDropB),int(yDropB)), (int(xPt3),int(yPt3)), (255,255,0),2)
                if xPt1 < xPt3+15 and xPt1 > xPt3-15 and yPt1 < yPt3+15 and yPt1 > yPt3-15  and c > 1:
                    c+=1
            if c < 4:
                if bestPath[1][0] == 1:
                    if c == 3:
                        cv2.line(frame, (int(xPt1), int(yPt1)), (int(xDropG),int(yDropG)), (255,255,0),2)
                    else:
                        cv2.line(frame, (int(xPt3), int(yPt3)), (int(xDropG),int(yDropG)), (255,255,0),2)
                    if xPt1 < xDropG+15 and xPt1 > xDropG-15 and yPt1 < yDropG+15 and yPt1 > yDropG-15 and c > 2:
                        c+=1
                else:
                    if c == 3:
                        cv2.line(frame, (int(xPt1), int(yPt1)), (int(xDropB),int(yDropB)), (255,255,0),2)
                    else:
                        cv2.line(frame, (int(xPt3), int(yPt3)), (int(xDropB),int(yDropB)), (255,255,0),2)
                    if xPt1 < xDropB+15 and xPt1 > xDropB-15 and yPt1 < yDropB+15 and yPt1 > yDropB-15 and c > 2:
                        c+=1
            if c < 5:
                if bestPath[1][0] == 1:
                    if c == 4:
                        cv2.line(frame, (int(xPt1),int(yPt1)), (int(xPt4),int(yPt4)), (255,0,0),2)
                    else:
                        cv2.line(frame, (int(xDropG),int(yDropG)), (int(xPt4),int(yPt4)), (255,0,0),2)
                else:
                    if c == 4:
                        cv2.line(frame, (int(xPt1),int(yPt1)), (int(xPt4),int(yPt4)), (255,0,0),2)
                    else:
                        cv2.line(frame, (int(xDropB),int(yDropB)), (int(xPt4),int(yPt4)), (255,0,0),2)
                if xPt1 < xPt4+15 and xPt1 > xPt4-15 and yPt1 < yPt4+15 and yPt1 > yPt4-15  and c > 3:
                    c+=1
            if c < 6:
                if bestPath[2][0] == 1:
                    if c == 5:
                        cv2.line(frame, (int(xPt1),int(yPt1)), (int(xDropG),int(yDropG)), (255,0,0),2)
                    else:
                        cv2.line(frame, (int(xPt4),int(yPt4)), (int(xDropG),int(yDropG)), (255,0,0),2)
                    if xPt1 < xDropG+15 and xPt1 > xDropG-15 and yPt1 < yDropG+15 and yPt1 > yDropG-15  and c > 4:
                        c+=1
                else:
                    if c == 5:
                        cv2.line(frame, (int(xPt1),int(yPt1)), (int(xDropB),int(yDropB)), (255,0,0),2)
                    else:
                        cv2.line(frame, (int(xPt4),int(yPt4)), (int(xDropB),int(yDropB)), (255,0,0),2)
                    if xPt1 < xDropB+15 and xPt1 > xDropB-15 and yPt1 < yDropB+15 and yPt1 > yDropB-15  and c > 4:
                        c+=1
            if c < 7:
                if bestPath[2][0] == 1:
                    if c == 6:
                        cv2.line(frame, (int(xDropG),int(yDropG)), (int(xPt5),int(yPt5)), (255,0,255),2)
                    else:
                        cv2.line(frame, (int(xDropG),int(yDropG)), (int(xPt5),int(yPt5)), (255,0,255),2)
                else:
                    if c == 6:
                        cv2.line(frame, (int(xPt1),int(yPt1)), (int(xPt5),int(yPt5)), (255,0,255),2)
                    else:
                        cv2.line(frame, (int(xDropB),int(yDropB)), (int(xPt5),int(yPt5)), (255,0,255),2)
                if xPt1 < xPt5+15 and xPt1 > xPt5-15 and yPt1 < yPt5+15 and yPt1 > yPt5-15  and c > 5:
                    c+=1
            if c < 8:
                if bestPath[3][0] == 1:
                    if c == 7:
                        cv2.line(frame, (int(xPt1),int(yPt1)), (int(xDropG),int(yDropG)), (255,0,255),2)
                    else:
                        cv2.line(frame, (int(xPt5),int(yPt5)), (int(xDropG),int(yDropG)), (255,0,255),2)
                    if xPt1 < xDropG+15 and xPt1 > xDropG-15 and yPt1 < yDropG+15 and yPt1 > yDropG-15 and c > 6:
                        c+=1
                else:
                    if c == 7:
                        cv2.line(frame, (int(xPt1),int(yPt1)), (int(xDropB),int(yDropB)), (255,0,255),2)
                    else:
                        cv2.line(frame, (int(xPt5),int(yPt5)), (int(xDropB),int(yDropB)), (255,0,255),2)
                    if xPt1 < xDropB+15 and xPt1 > xDropB-15 and yPt1 < yDropB+15 and yPt1 > yDropB-15 and c > 6:
                        c+=1
            if c < 9:
                if bestPath[3][0] == 1:
                    if c == 8:
                        cv2.line(frame, (int(xPt1),int(yPt1)), (int(xHome),int(yHome)), (0,0,255),2)
                    else:
                        cv2.line(frame, (int(xDropG),int(yDropG)), (int(xHome),int(yHome)), (0,0,255),2)
                else:
                    if c == 8:
                        cv2.line(frame, (int(xPt1),int(yPt1)), (int(xHome),int(yHome)), (0,0,255),2)
                    else:
                        cv2.line(frame, (int(xDropB),int(yDropB)), (int(xHome),int(yHome)), (0,0,255),2)
                if xPt1 < xHome+15 and xPt1 > xHome-15 and yPt1 < yHome+15 and yPt1 > yHome-15 and c > 7:
                    c+=1








        

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