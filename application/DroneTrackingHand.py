from djitellopy.tello import Tello 
from threading import Thread
import cv2
import numpy as np
import keyboard
import time
from cvzone.HandTrackingModule import HandDetector 
import cvzone

class PID_control():
    
    def __init__(self, P, I, D, out_limit = 100, I_limit = 10):
        
        # 比例、積分、微分參數
        self.Kp = P
        self.Ki = I
        self.Kd = D
        
        self.I_limit = I_limit # 累積誤差上限
        self.out_limit = out_limit # 輸出限制
        self.clear()
        
    def clear(self):
        # 積分、微分所需變數
        self.last_error = 0.0 
        self.sum_error = 0.0 
        
        # 輸出
        self.output = 0.0
        
    def PID(self, obj_val, real_val):
        
        # error誤差
        error = obj_val - real_val
        
        # 積分限幅
        self.sum_error+=error
        if self.Ki*self.sum_error>=self.I_limit:
            self.sum_error = self.I_limit/self.Ki
        elif self.Ki*self.sum_error<=-self.I_limit:
            self.sum_error = -self.I_limit/self.Ki
            
        self.output = self.Kp*error + self.Ki*self.sum_error + self.Kd*(error-self.last_error)
        self.output = int(np.clip(self.output,-self.out_limit,self.out_limit)) 
        
        self.last_error = error
        
        return self.output
     
class TelloDrone(Tello):
    
    def __init__(self):
        Tello.__init__(self)
        self.connect()
        print("battery: ",self.get_battery())
        
        # keyboard control
        self.control_speed = [0,0,0,0] 
        self.keyboard_thread = Thread(target=self.getKeyboardInput)
        self.keyboard_thread.start()
        
        # hand detection
        self.detector = HandDetector(detectionCon=0.8, maxHands=1)
        
        # pid
        self.rotate_pid = PID_control(0.2,0.02,0.9) 
        self.updown_pid = PID_control(0.2,0.02,0.7) 
        self.frontback_pid = PID_control(0.5,0.02,0.7) 
        self.scale = 0.1
        
        # drone video
        self.streamon() 
        self.cap = self.get_frame_read()
        self.drone_frame()
        

    def getKeyboardInput(self):

        while True:
            self.control_speed = [0,0,0,0]
            speed = 50
            # 左右
            if keyboard.is_pressed("LEFT"): self.control_speed[0]=-speed
            elif keyboard.is_pressed("RIGHT"): self.control_speed[0]= speed
            
            # 前後
            if keyboard.is_pressed("UP"): self.control_speed[1]= speed
            elif keyboard.is_pressed("DOWN"): self.control_speed[1]=-speed
            
            # 上下
            if keyboard.is_pressed("w"): self.control_speed[2]= speed
            elif keyboard.is_pressed("s"): self.control_speed[2]=-speed
            
            # 旋轉
            if keyboard.is_pressed("a"): self.control_speed[3]=-speed
            elif keyboard.is_pressed("d"): self.control_speed[3]= speed
            
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
            
            
            
            time.sleep(0.05) 
    
    def trackHand(self,fb_speed, ud_speed, yv_speed):       
        self.control_speed[1] = fb_speed 
        self.control_speed[2] = ud_speed 
        self.control_speed[3] = yv_speed 

        
    def drone_frame(self):
        pTime = 0
        while True:
            img = self.cap.frame
            h, w, _ = img.shape
            
            # fly keyboard control
            self.send_rc_control(self.control_speed[0],self.control_speed[1],self.control_speed[2],self.control_speed[3])
            
            # hand detect
            img = self.detector.findHands(img)
            lmList, hand = self.detector.findPosition(img) 

            if lmList:
                bbox = hand['bbox'] 
                cv2.circle(img, lmList[9], 15, (0, 255, 0), cv2.FILLED)
                cv2.rectangle(img, (bbox[0],bbox[1]), (bbox[0]+bbox[2],bbox[1]+bbox[3]), (255, 0, 0), 2)
                
                # PID control
                yv_speed = self.rotate_pid.PID(w//2, lmList[9][0])
                ud_speed = self.updown_pid.PID(h//2, lmList[9][1])
                fb_speed = self.frontback_pid.PID(w*self.scale, bbox[2])
                self.trackHand(fb_speed,ud_speed,-yv_speed)
            else:
                self.control_speed[0] = 0
                self.control_speed[1] = 0
                self.control_speed[2] = 0
                self.control_speed[3] = 0
            
            # fps
            cTime = time.time()
            fps = 1/(cTime-pTime)
            pTime = cTime
            cv2.putText(img, str(int(fps)), (20,50),cv2.FONT_HERSHEY_PLAIN, 3,(255, 0, 0), 3)
            
            cv2.imshow('frame',img)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        self.streamoff()
        cv2.destroyAllWindows()
        self.end()
        
        
if __name__ == '__main__':
    TelloDrone()
    
