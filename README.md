# tello教學
介紹Tello無人機基本功能，包括：

1. 起飛降落
2. 控制無人機
3. 獲取無人機影像

介紹如何應用於影像處理，包括：

1. 姿態估計
2. 手心追蹤
3. 路徑地圖

## 設置
1. 安裝
```
mediapipe==0.8.7.1
cvzone==1.4.1
keyboard==0.13.5
opencv-python==4.4.0.44
```
其中cvzone版本需要正確，1.5後的版本經過改動

2. 連接Tello

開啟無人機電源(左側按紐)並連接到wifi

<img width="346" alt="wifi_connection" src="https://user-images.githubusercontent.com/13486777/110932822-a7b30f00-8334-11eb-9759-864c3dce652d.png">

## 基本功能
1. 起飛降落
    
    ```python
    from djitellopy.tello import Tello

    mydrone = Tello()
    mydrone.connect() # 連線
    print(mydrone.get_battery()) # 顯示電量

    mydrone.takeoff() # 起飛
    mydrone.move_forward(30) # 前進30cm
    mydrone.rotate_counter_clockwise(90) # 逆時鐘旋轉90度
    mydrone.land() # 降落
    ```

3. 控制無人機
    
    讀取鍵盤控制無人機，另外開一條執行緒防止影響到後續的影像
    
    控制方式可參考[DJITelloPy API](https://djitellopy.readthedocs.io/en/latest/tello/)，主要分為幾種控制方法：
    * 控制速度：控制三軸與旋轉速度，速度為cm/s
    
        ```python
        send_rc_control(self, left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)
        ```
    * 移動距離：Tello的上下左右前後、順逆時鐘移動距離，單位為cm
    
        ```python
        move_up(self, x)
        move_down(self, x)
        move_left(self, x)
        move_right(self, x)
        move_forward(self, x)
        move_back(self, x)
        rotate_clockwise(self, x)
        rotate_counter_clockwise(self, x)
        ```
    * 前往座標：移動到相對於當前位置的xyz，速度為cm/s
    
        ```python
        go_xyz_speed(self, x, y, z, speed)
        ```
    * 弧形前往座標：通過x1,y1,z1前往x2,y2,z2，速度為cm/s

        ```python
        curve_xyz_speed(self, x1, y1, z1, x2, y2, z2, speed)
        ```
        
    範例**DroneControl.py**使用*控制速度*的方式來控制無人機
    
3. 獲取無人機影像
    參考範例**DroneFrame.py**

## 影像處理應用
1. 姿態估計: 預測人體骨架關鍵點位置
2. 手心追蹤: 預測手部姿態、並以PID做單目標追蹤
3. 路徑地圖: 無人機移動軌跡映射到世界地圖顯示
