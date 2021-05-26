from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil


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
vehicle.simple_takeoff(8)  # 无人机起飞到8m
time.sleep(5)
mav_movements(5, 0, 0)  # 无人机前移5m
mav_movements(0, 5, 0)  # 无人机右移5m
mav_movements(0, 0, 1)  # 无人机下移1m
mav_movements(-5, 0, 0)  # 无人机后移5m
mav_movements(0, -5, 0)  # 无人机左移5m
mav_movements(0, 0, -2)  # 无人机上移2m
vehicle.mode = VehicleMode("LAND")  # 无人机降落
