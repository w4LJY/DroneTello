from djitellopy.tello import Tello 
import keyboard
from threading import Thread
import time 
       

def getKeyboardInput():
    global lr,fb,ud,yv
    while True:
        lr, fb, ud, yv = 0,0,0,0
        speed = 50
        # 左右
        if keyboard.is_pressed("LEFT"): lr=-speed
        elif keyboard.is_pressed("RIGHT"): lr= speed
        
        # 前後
        if keyboard.is_pressed("UP"): fb= speed
        elif keyboard.is_pressed("DOWN"): fb=-speed
        
        # 上下
        if keyboard.is_pressed("w"): ud= speed
        elif keyboard.is_pressed("s"): ud=-speed
        
        # 旋轉
        if keyboard.is_pressed("a"): yv=-speed
        elif keyboard.is_pressed("d"): yv= speed
        
        # 降落
        if keyboard.is_pressed("q"): mydrone.land(); time.sleep(3) 
        
        # 起飛
        if keyboard.is_pressed("e"): mydrone.takeoff(); 
        
        time.sleep(0.05)

if __name__ == '__main__':
    mydrone = Tello()
    mydrone.connect() # 連線
    print(mydrone.get_battery()) # 顯示電量
    
    global lr,fb,ud,yv
    lr,fb,ud,yv = 0,0,0,0
    
    keyboard_thread = Thread(target=getKeyboardInput).start()
    
    while True:
        mydrone.send_rc_control(lr,fb,ud,yv)
        
