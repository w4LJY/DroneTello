from djitellopy.tello import Tello
import cv2

mydrone = Tello()
mydrone.connect() 
print(mydrone.get_battery()) 


mydrone.streamon() 
cap = mydrone.get_frame_read()
while True:
    img = cap.frame 
    
    #img = cv2.resize(img,(360,240))
    
    cv2.imshow("frame",img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

mydrone.streamoff()
cv2.destroyAllWindows()