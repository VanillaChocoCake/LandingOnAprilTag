# LandingOnApriltag
# 简介

本文档用于介绍大创项目”基于视觉导航的自主飞行无人机研发“。

# 项目环节

本项目主要分为两个环节：图像处理与无人机控制，下面将一一进行介绍

## 图像处理

图像处理基于[AprilTag](https://april.eecs.umich.edu/software/apriltag.html)与[OpenCV](https://opencv.org)

## 无人机控制

在此部分主要用到了[DroneKit](http://dronekit.io)与一部分MAVLink命令   
所有MAVLink命令可以参考此[界面](https://mavlink.io/zh/messages/common.html)   
更加详细的API介绍可参考此[页面](https://dronekit-python.readthedocs.io/en/latest/automodule.html)   

### 相关MAVLink命令在DroneKit中的使用

---

#### 无人机连接与解锁

---

```python
vehicle = connect("/dev/ttyUSB0", baud=57600, wait_ready=False) 
# 设定波特率和接口
vehicle.wait_ready(True, timeout=300) 
# 不能在上一步设定wait_ready=True的原因为，connect函数的超时时间为30s，
# 而实际中需要一分钟进行连接
while not vehicle.is_armable:  # 判断是否可解锁，若不可解锁，则等待其初始化完成
    print("init")
    time.sleep(1)
vehicle.mode = VehicleMode("GUIDED") # 设定引导模式进行起飞
vehicle.armed = True
while not vehicle.armed:  # 等待其解锁完毕
    print("解锁中...")
    time.sleep(1)
```

#### 无人机起飞

---

DroneKit内置函数可以让无人机起飞到相对于Home的指定高度附近

```python
vehicle.simple_takeoff(desired_altitude)
```

#### 无人机移动

---

坐标系选取为以无人机为原点，正方向分别为前方，右方以及下方，MAVLink Message为MAV_FRAME_BODY_OFFSET_NED。
指令生成：

```python
msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, # 0就可以
            0, 0, # 0就可以
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b110111111000,  # 该字符串为使能距离，使能速度为0b110111000111，
# 使能速度和距离为0b110111000000
            x,  y,  z,  # 正方向的移动距离，单位为米
            vx, vy, vz, # 正方向的速度，单位为m/s
            ax, ay, az, # 加速度，不支持，置0
            yaw, yaw_rate # 偏航角（0代表正前方），一般置0；角速度，一般置0
        )
```

通过指定使能方式，可以按照无人机向六个方向进行移动（前、后、左、右、上、下）。
指令发送：

```python
vehicle.send_mavlink(msg)
vehicle.flush()
```

#### 无人机改变航向

---

相关MAVLink命令：MAV_CMD_CONDITION_YAW

```python
msg = vehicle.message_factory.command_long_encode(
            0, 0, # 目标系统，目标内容，一般可以置0
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0, # 确认号
            Angle, # 角度（角度值）
            Angular Speed,  # 角速度
            Direction, # 方向: -1: 逆时针, 1: 顺时针
            1, # 1表示以无人机朝向为0度，0表示以北为0度
            0, 0, 0)
vehicle.send_mavlink(msg)
vehicle.flush()
```

#### 无人机前往指定位置

---

DroneKit自带的simple_goto函数可以满足预期目标

```python
from dronekit import LocationGlobalRelative # 使用相对高度
from dronekit import LocationGlobal # 使用绝对高度

... # (连接到无人机等流程)

home_lat = vehicle.location.global_relative_frame.lat
home_lon = vehicle.location.global_relative_frame.lon
home_point = LocationGlobalRelative(home_lat, home_lon, 4) # 经纬度以及高度
vehicle.simple_goto(home_point)
```

#### 无人机任务

---

任务添加流程：

```python
cmds = vehicle.commands
cmds.download()
cmds.wait_ready() #等待命令下载完成
cmds.clear() # 清除所有命令，写入新的
lat = -34.364114
lon = 149.166022
altitude = 30.0 # 写入经纬度与相对高度
cmd = Command(0,0,0,
mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
0, 0, 0, 0, 0, 0,lat, lon, altitude) # 命令生成
cmds.add(cmd) # 命令添加
... # 可以进行若干次类似操作
cmds.upload() # 命令上传
```

Command函数参数：

```python
cmd = Command(0,0,0,
mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
0, 0, 0, 0, 0, 0,lat, lon, altitude)
```

- 目标系统：可以设为任意值
- 目标内容：多数情况下设为0
- 序号：设为0，因为API会自动编号
- 坐标系：可以选择多种坐标系，具体请参考此[界面](https://mavlink.io/zh/messages/common.html)，示例为相对坐标系
- 命令：可以选择多种命令，示例为导航至预定地点的命令
- current：设为0
- 自动继续：设为0（不支持）
- 剩余参数为MAVLink命令的参数，具体请参考此[界面](https://mavlink.io/zh/messages/common.html)，在此不一一赘述

#### 无人机刹车

---

```python
vehicle.mode = VehicleMode("BRAKE")
```

#### 无人机降落

---

```python
vehicle.mode = VehicleMode("LAND")
```

