# ESP32 Mecanum Wheel Car Kit
![cover esp32](https://github.com/user-attachments/assets/70b5a9f8-4b43-48bd-a78f-962893cc673e)
本项目基于ESP32开发板，利用ESP-NOW协议实现无线遥控功能。系统采用L298N电机驱动板控制四个麦克纳姆轮，通过PS2摇杆进行运动控制。  
项目设计了两种控制模式：连续模式下，摇杆摇动方向和幅度直接映射为小车的转向角度和速度，实现运动方向修正；离散模式下，通过划分摇杆控制区域，触发麦克纳姆轮特有的全向运动模式（前后移动、左右平移和斜向移动）。  
代码结构清晰，包含完整的硬件连接说明和详细的参数配置，便于二次开发和定制。  
aranre.ino文件为c++版本的接收端程序文件  
arantr.ino文件为c++版本的发送端程序文件  
threlast.py文件为python版本的接收端程序文件  
thtrlast.py文件为python版本的发送端程序文件  

