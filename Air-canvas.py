import numpy as np
import cv2
from collections import deque
def setup():
    pass
cv2.namedWindow("aircanvas")
cv2.createTrackbar("upper_hue","aircanvas",153,180,setup)
cv2.createTrackbar("lower_hue","aircanvas",64,180,setup)
cv2.createTrackbar("upper_value","aircanvas",255,255,setup)
cv2.createTrackbar("lower_value","aircanvas",49,255,setup)
cv2.createTrackbar("upper_saturation","aircanvas",255,255,setup)
cv2.createTrackbar("lower_saturation","aircanvas",72,255,setup)
blue_points=[deque(maxlen=1024)]
red_points=[deque(maxlen=1024)]
green_points=[deque(maxlen=1024)]
yellow_points=[deque(maxlen=1024)]
blue_index=0
red_index=0
green_index=0
yellow_index=0
k_nel=np.ones((5,5),np.uint8)
colors = [(255, 0, 0), (0, 255, 0),
          (0, 0, 255), (0, 255, 255)]
color_index=0
paintWindow = np.zeros((471, 636, 3)) + 255
cv2.namedWindow("paint",cv2.WINDOW_AUTOSIZE)
capture=cv2.VideoCapture(0)
while True:
    ret,frame=capture.read()
    frame=cv2.flip(frame,1)
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    u_hue=cv2.getTrackbarPos("upper_hue","aircanvas")
    u_value=cv2.getTrackbarPos("upper_value","aircanvas")
    u_satu=cv2.getTrackbarPos("upper_saturation","aircanvas")
    l_hue=cv2.getTrackbarPos("lower_hue","aircanvas")
    l_value=cv2.getTrackbarPos("lower_value","aircanvas")
    l_satu=cv2.getTrackbarPos("lower_saturation","aircanvas")
    uppper=np.array([u_hue,u_satu,u_value])
    lower=np.array([l_hue,l_satu,l_value])
    frame = cv2.rectangle(frame, (40, 1), (140, 65),
                          (122, 122, 122), -1)
    frame = cv2.rectangle(frame, (160, 1), (255, 65),
                          colors[0], -1)
    frame = cv2.rectangle(frame, (275, 1), (370, 65),
                          colors[1], -1)
    frame = cv2.rectangle(frame, (390, 1), (485, 65),
                          colors[2], -1)
    frame = cv2.rectangle(frame, (505, 1), (600, 65),
                          colors[3], -1)

    cv2.putText(frame, "CLEAR ALL", (49, 33),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 2, cv2.LINE_AA)

    cv2.putText(frame, "BLUE", (185, 33),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 2, cv2.LINE_AA)

    cv2.putText(frame, "GREEN", (298, 33),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 2, cv2.LINE_AA)

    cv2.putText(frame, "RED", (420, 33),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 2, cv2.LINE_AA)

    cv2.putText(frame, "YELLOW", (520, 33),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (150, 150, 150), 2, cv2.LINE_AA)
    mask=cv2.inRange(hsv,lower,uppper)
    mask=cv2.erode(mask,k_nel,iterations=1)
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,k_nel)
    mask=cv2.dilate(mask,k_nel,iterations=1)
    cnt,_=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if len(cnt)>0:
        cnt=sorted(cnt,key=cv2.contourArea,reverse=True)[0]
        ((x,y),radiuss)=cv2.minEnclosingCircle(cnt)
        cv2.circle(frame,(int(x),int(y)),int(radiuss),(0,255,255),2)
        m=cv2.moments(cnt)
        center=(int(m["m10"]/m["m00"])),(int(m["m01"]/m["m00"]))
        if center[1]<=65:
            if 40<=center[0]<=140:
                blue_points=[deque(maxlen=512)]
                red_points=[deque(maxlen=512)]
                green_points=[deque(maxlen=512)]
                yellow_points=[deque(maxlen=512)]
                blue_index=0
                red_index=0
                green_index=0
                yellow_index=0
                paintWindow[67:,:,:]=255
            elif 160<=center[0]<=255:
                color_index=0
            elif 275<=center[0]<=370:
                color_index=1
            elif 390<=center[0]<=485:
                color_index=2
            elif 505<=center[0]<=600:
                color_index=3
        else:
            if color_index == 0:
                blue_points[blue_index].appendleft(center)
            elif color_index == 1:
                green_points[green_index].appendleft(center)
            elif color_index == 2:
                red_points[red_index].appendleft(center)
            elif color_index == 3:
                yellow_points[yellow_index].appendleft(center)
    else:
        blue_points.append(deque(maxlen=512))
        blue_index += 1
        green_points.append(deque(maxlen=512))
        green_index += 1
        red_points.append(deque(maxlen=512))
        red_index += 1
        yellow_points.append(deque(maxlen=512))
        yellow_index += 1
    line=[blue_points,green_points,red_points,yellow_points]
    for i in range(len(line)):
        for j in range(len(line[i])):
            for k in range(len(line[i][j])):
                if line[i][j][k] is None or line[i][j][k-1] is None:
                    continue
                cv2.line(frame,line[i][j][k-1],line[i][j][k],colors[i],2)
                cv2.line(paintWindow, line[i][j][k - 1], line[i][j][k], colors[i], 2)
    cv2.imshow("Frame",frame)
    cv2.imshow("paint",mask)
    if cv2.waitKey(1)==13:
        break
capture.release()
cv2.destroyAllWindows()