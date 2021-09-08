from djitellopy.tello import Tello 
from threading import Thread
import cv2
import numpy as np
import keyboard
import time
import math
    
class TelloDrone(Tello):
    
    def __init__(self):
        Tello.__init__(self)
        self.connect()
        print("battery: ",self.get_battery())
              
        # keyboard control
        self.speed = 50
        self.control_speed = [0,0,0,0] 
        self.keyboard_thread = Thread(target=self.getKeyboardInput)
        self.keyboard_thread.start()
        
        # mapping
        fSpeed = 250/10 
        aSpeed = 360/10
        self.interval = 0.25
        
        self.dInterval = fSpeed*self.interval
        self.aInterval = aSpeed*self.interval
        self.angle = 0 
        self.yaw = 0
        self.map_x,self.map_y = 500,500
        self.points = [] # 儲存走過的路徑
        
        # drone video
        self.streamon() 
        self.cap = self.get_frame_read()
        self.drone_frame()
        

    def getKeyboardInput(self):

        while True:
            self.control_speed = [0,0,0,0]
            speed = 50
            self.dist = 0
            
            # 左右
            if keyboard.is_pressed("LEFT"): 
                self.control_speed[0]=-speed
                self.dist = self.dInterval
                self.angle = -180
            elif keyboard.is_pressed("RIGHT"): 
                self.control_speed[0]= speed
                self.dist = -self.dInterval
                self.angle = 180
            
            # 前後
            if keyboard.is_pressed("UP"): 
                self.control_speed[1]= speed
                self.dist = self.dInterval
                self.angle = 270
                
            elif keyboard.is_pressed("DOWN"): 
                self.control_speed[1]=-speed
                self.dist = -self.dInterval
                self.angle = -90
            
            # 上下
            if keyboard.is_pressed("w"): self.control_speed[2]= speed
            elif keyboard.is_pressed("s"): self.control_speed[2]=-speed
            
            # 旋轉
            if keyboard.is_pressed("a"): 
                self.control_speed[3]=-speed
                self.yaw -= self.aInterval
            elif keyboard.is_pressed("d"): 
                self.control_speed[3]= speed
                self.yaw += self.aInterval
            
            # 降落
            if keyboard.is_pressed("q"): 
                self.control_speed[0] = 0
                self.control_speed[1] = 0
                self.control_speed[2] = 0
                self.control_speed[3] = 0
                self.land()
                time.sleep(3) 
            
            # 起飛
            if keyboard.is_pressed("e"): self.takeoff()
            
            # flip 
            if keyboard.is_pressed("j"): self.flip_left(); time.sleep(1)
            elif keyboard.is_pressed("l"): self.flip_right(); time.sleep(1)
            elif keyboard.is_pressed("i"): self.flip_forward(); time.sleep(1)
            elif keyboard.is_pressed("k"): self.flip_back(); time.sleep(1)
            
            # 退出
            if keyboard.is_pressed("ESC"): self.land(); time.sleep(3); self.end() 
            
            time.sleep(self.interval) 
            
            self.angle +=self.yaw
            self.map_x +=int(self.dist*math.cos(math.radians(self.angle)))
            self.map_y +=int(self.dist*math.sin(math.radians(self.angle)))
  
    def drawPoints(self,img):
        self.points.append([self.map_x,self.map_y])
        for point in self.points:
            cv2.circle(img,point, 5, (0,0,255),cv2.FILLED)
            
        cv2.circle(img,self.points[-1], 6, (0,255,0),cv2.FILLED)
        
    def drone_frame(self):
        pTime = 0
        while True:
            img = self.cap.frame
            h, w, _ = img.shape
            
            # fly keyboard control
            self.send_rc_control(self.control_speed[0],self.control_speed[1],self.control_speed[2],self.control_speed[3])
            
            # mapping
            drawFrame = np.zeros((1000,1000,3), np.uint8)
            self.drawPoints(drawFrame)
            
            # fps
            cTime = time.time()
            fps = 1/(cTime-pTime)
            pTime = cTime
            cv2.putText(img, str(int(fps)), (20,50),cv2.FONT_HERSHEY_PLAIN, 3,(255, 0, 0), 3)
            
            cv2.imshow('frame',img)
            cv2.imshow('drawframe',drawFrame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        self.streamoff()
        cv2.destroyAllWindows()
        self.end()
        
        
if __name__ == '__main__':
    TelloDrone()
    
