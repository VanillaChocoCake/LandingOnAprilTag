from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import numpy as np

Return_Land = 0
Can_Land = 0
# 标识是否已经降落过的变量
Can_RTL = 0
# 标识是否已经在回到出发点的路上的变量（虽然不会用到RTL模式
# connection_string = '127.0.0.1:14551'
# vehicle = connect(connection_string, wait_ready=False)
######################################################################
# 连接到无人机，并使其起飞 #
######################################################################
vehicle = connect("/dev/ttyUSB0", baud=57600, wait_ready=False)
vehicle.wait_ready(True, timeout=300)
while not vehicle.is_armable:  # 判断是否可解锁，若不可解锁，则等待其初始化完成
    print("init")
    time.sleep(1)
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:  # 等待其解锁完毕
    print("解锁中...")
    time.sleep(1)
print("无人机成功解锁，准备起飞...")
vehicle.groundspeed = 0.1
vehicle.airspeed = 0.1
first_detect = 1  # 一个判断是否是第一次收到来自图传处理完毕的数据的变量
data_path = "/path/DataTransmission/data.txt"
home_lat = vehicle.location.global_relative_frame.lat
home_lon = vehicle.location.global_relative_frame.lon
home_point = LocationGlobalRelative(home_lat, home_lon, 4)
print(vehicle.location.global_relative_frame.alt)


def mav_movements(forward, right, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b110111111000,  # enable distance, velocity is also an option
        forward / 100, right / 100, down / 100,  # cm -> m
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


vehicle.simple_takeoff(3)
# 让其起飞

time.sleep(2.8)
vehicle.mode = VehicleMode("BRAKE")
time.sleep(3)
# 等待中断后无人机稳定下来
vehicle.mode = VehicleMode("GUIDED")
# mav_movements(500, 0, 0)
# time.sleep(8)
# 无人机稳定后切换到GUIDED模式
# vehicle.simple_goto()


while True:
    print(vehicle.location.global_relative_frame.alt)
    try:
        f = open(data_path, 'r')
        lines = f.readlines()
        latest_data = lines[-1]
        degree = int(latest_data.split(" ")[0])
        Horizontal_Distance = int(latest_data.split(" ")[1])
        Delta_Height = int(latest_data.split(" ")[2])
        # 从cpp处理的数据文件中读取数据
    except:
        continue
    if not vehicle.armed \
            and Can_Land == 1 \
            and Can_RTL == 0:  # 意味着无人机已经在目的地降落过，现在准备返航
        while not vehicle.is_armable:
            print("init")
            time.sleep(1)
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)
    if degree == -666:  # 无人机飞行过程中发生一些意外，因此被人手动终端，换成遥控器
        # vehicle.mode = VehicleMode("BRAKE")
        vehicle.mode = VehicleMode("LAND")
        print("紧急情况！已切换到遥控器控制！")
        Can_RTL = 1
        # 因为发生意外了，所以默认进入最终的阶段，不再执行相关定点降落的命令
        break
    if first_detect:  # 首次收到数据
        vehicle.mode = VehicleMode("BRAKE")
        # 为了防止飞过头，因此中断现在正在进行的行为
        time.sleep(3)
        # 等待无人机稳定下来
        vehicle.mode = VehicleMode("GUIDED")
        # 切换回GUIDED模式，准备执行命令
        print("正在准备定点降落")
        first_detect = 0
        time.sleep(1)
        continue
        # 回到循环的开始，读取最新数据

    if Delta_Height <= 150 \
            and Horizontal_Distance <= 15 \
            and Can_Land == 0 \
            and Can_RTL == 0:
        # 当相对高度差距较小，水平距离差距不大时，可以进行降落
        Can_Land = 1
        vehicle.mode = VehicleMode("LAND")
        print("降落中...")
        time.sleep(60)
        # 等待60s降落完成以及拿到物品
        # continue
        break  # test! test! test!

    # 优先级：水平距离>高度
    if Delta_Height >= 200 \
            and Horizontal_Distance > 40 \
            and Can_RTL == 0 \
            and Can_Land == 0:
        mav_movements(int(1.2 * (Horizontal_Distance - 40) * np.cos(degree * np.pi / 180)),
                      int(-1 * 1.2 * (Horizontal_Distance - 40) * np.sin(degree * np.pi / 180)), 0)
        print("已经前移约", int(1.2 * (Horizontal_Distance - 40) * np.cos(degree * np.pi / 180)), "cm，右移",
              int(-1 * 1.2 * (Horizontal_Distance - 40) * np.sin(degree * np.pi / 180)), "cm")
        time.sleep(4)
        continue
    if Horizontal_Distance > 15 \
            and Delta_Height <= 200 \
            and Can_Land == 0 \
            and Can_RTL == 0:
        mav_movements(int(1.2 * (Horizontal_Distance - 15) * np.cos(degree * np.pi / 180)),
                      int(-1 * 1.2 * (Horizontal_Distance - 15) * np.sin(degree * np.pi / 180)), 0)
        # 角度的分量还是要乘一下，不然误差可能更大
        print("已经前移约", int(1.2 * (Horizontal_Distance - 15) * np.cos(degree * np.pi / 180)), "cm，右移",
              int(-1 * 1.2 * (Horizontal_Distance - 15) * np.sin(degree * np.pi / 180)), "cm")
        time.sleep(4)
        continue
    # 虽然没必要把约束条件写的那么全，但为了方便理解还是写一下

    if Can_Land == 0 \
            and Can_RTL == 0:
        if Delta_Height >= 300:
            mav_movements(0, 0, 100)
            print("下移1m")
        else:
            mav_movements(0, 0, 70)
            print("下移0.7m")
        time.sleep(4)
        continue

    if Can_Land == 1 \
            and Can_RTL == 0:
        vehicle.simple_takeoff(5)
        # vehicle.simple_goto(home_lat, home_lon, 5)
        Can_RTL = 1
        print("返回")
        # break
    if Can_RTL == 1 \
            and Return_Land == 0:
        if np.abs(vehicle.location.global_relative_frame.lat - home_lat) < 1e-5 \
                and np.abs(vehicle.location.global_relative_frame.lon - home_lon) < 1e-5:
            vehicle.mode = VehicleMode("BRAKE")
            time.sleep(4)
            vehicle.mode = VehicleMode("LAND")
            Return_Land = 1
vehicle.close()
