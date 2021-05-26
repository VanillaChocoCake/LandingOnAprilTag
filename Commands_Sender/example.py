from dronekit import connect, VehicleMode, Command
import time
from pymavlink import mavutil

CW = -1
CCW = 1
vehicle = connect("/dev/ttyUSB0", baud=57600, wait_ready=False)
vehicle.wait_ready(True, timeout=300)


def arm_and_takeoff():
    while not vehicle.is_armable:  # 判断是否可解锁，若不可解锁，则等待其初始化完成
        print("init")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:  # 等待其解锁完毕
        print("解锁中...")
        time.sleep(1)
    vehicle.simple_takeoff(8)
    time.sleep(10)


def mav_movements(forward, right, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b110111111000,  # enable distance, velocity is also an option
        forward, right, down,  # cm -> m
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    time.sleep(5)


def change_yaw(Angle, Direction):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # 目标系统，目标内容，一般可以置0
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,  # 确认号
        Angle,  # 角度（角度值）
        0,  # 角速度
        Direction,  # 方向: -1: 逆时针, 1: 顺时针
        1,  # 1表示以无人机朝向为0度，0表示以北为0度
        0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


arm_and_takeoff()
# Part 1
vehicle.simple_takeoff(8)  # 无人机起飞到8m
time.sleep(5)
mav_movements(5, 0, 0)  # 无人机前移5m
mav_movements(0, 5, 0)  # 无人机右移5m
mav_movements(0, 0, 1)  # 无人机下移1m
mav_movements(-5, 0, 0)  # 无人机后移5m
mav_movements(0, -5, 0)  # 无人机左移5m
mav_movements(0, 0, -2)  # 无人机上移2m
change_yaw(60, CW)  # 顺时针旋转60度
change_yaw(30, CCW)  # 逆时针旋转30度
vehicle.mode = VehicleMode("LAND")  # 无人机降落
time.sleep(10)

# Part 2
arm_and_takeoff()
home_lat = vehicle.location.global_relative_frame.lat
home_lon = vehicle.location.global_relative_frame.lon
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
cmds.clear()
desired_lat = input()
desired_lon = input()
cmd = Command(0, 0, 0,
              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
              mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
              0, 0, 0, 0, 0, 0, desired_lat, desired_lon, 10)
cmds.add(cmd)
cmd = Command(0, 0, 0,
              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
              mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
              0, 0, 0, 0, 0, 0, home_lat, home_lon, 5)
cmds.add(cmd)
cmds.upload()
vehicle.commands.next = 0
vehicle.mode = VehicleMode("AUTO")
next_point = vehicle.commands.next
while next_point < len(vehicle.commands):
    if vehicle.commands.next > next_point:
        next_point = vehicle.commands.next
    time.sleep(1)
vehicle.mode = VehicleMode("LAND")


