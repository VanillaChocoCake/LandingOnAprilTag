import cv2
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

cap = cv2.VideoCapture(0)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
# connection_string = '127.0.0.1:14551'
# vehicle = connect(connection_string, wait_ready=False)
vehicle = connect("/dev/ttyUSB0", baud=57600, wait_ready=False)
vehicle.wait_ready(True, timeout=300)
while not vehicle.is_armable:
    print("init")
    time.sleep(1)
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)
print("arm success")
vehicle.airspeed = 1.0
vehicle.groundspeed = 1.0
vehicle.simple_takeoff(3)
time.sleep(2.5)
vehicle.mode = VehicleMode("BRAKE")
time.sleep(1)
vehicle.mode = VehicleMode("GUIDED")
"""
while True:
    print(vehicle.location.global_relative_frame.lat, " ", vehicle.location.global_relative_frame.lon)
    time.sleep(1)
"""
while True:
    ret, frame = cap.read()
    cmd = cv2.waitKey(10)
    cv2.imshow("frame", frame)
    print(vehicle.location.global_relative_frame.alt)
    if vehicle.mode == "BRAKE":
        vehicle.mode = VehicleMode("GUIDED")
    if cmd == ord("w"):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            1, 1,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b110111111000,  # enable distance, velocity is also an option
            500 / 100, 0, 0,  # cm -> m
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        print("forward")
    elif cmd == ord("s"):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            1, 1,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b110111111000,  # enable distance, velocity is also an option
            -100 / 100, 0, 0,  # cm -> m
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        print("back")
    elif cmd == ord("a"):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            1, 1,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b110111111000,  # enable distance, velocity is also an option
            0, -100 / 100, 0,  # cm -> m
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        print("left")
    elif cmd == ord("d"):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            1, 1,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b110111111000,  # enable distance, velocity is also an option
            0, 100 / 100, 0,  # cm -> m
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        print("right")
    elif cmd == ord("i"):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            1, 1,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b110111111000,  # enable distance, velocity is also an option
            0, 0, -50 / 100,  # cm -> m
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        print("up")
    elif cmd == ord("j"):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            1, 1,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b110111111000,  # enable distance, velocity is also an option
            0, 0, 50 / 100,  # cm -> m
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        print("down")
    elif cmd == ord("n"):
        msg = vehicle.message_factory.command_long_encode(
            1, 1,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            30,
            0,  # yaw speed
            -1,
            1,
            0, 0, 0)
        vehicle.send_mavlink(msg)
        vehicle.flush()
        print("left 30 degree")
    elif cmd == ord("m"):
        msg = vehicle.message_factory.command_long_encode(
            1, 1,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            30,
            0,  # yaw speed
            1,
            1,
            0, 0, 0)
        vehicle.send_mavlink(msg)
        vehicle.flush()
        print("right 30 degree")
    elif cmd == ord("u"):
        if not vehicle.armed:
            while not vehicle.is_armable:
                print("init")
                time.sleep(1)
            vehicle.mode = VehicleMode("GUIDED")
            vehicle.armed = True
            while not vehicle.armed:
                print(" Waiting for arming...")
                time.sleep(1)
        vehicle.simple_takeoff(5)
        print("taking off")
    elif cmd == ord("r"):
        vehicle.mode = VehicleMode("RTL")
        print("RTL")
    elif cmd == ord("l"):
        vehicle.mode = VehicleMode("LAND")
        print("landing")
    elif cmd == ord("b"):
        if not vehicle.mode == "RTL" or not vehicle.mode == "LAND":
            vehicle.mode = VehicleMode("BRAKE")
            # vehicle.mode = VehicleMode("BRAKE")
        # vehicle.mode = VehicleMode("RTL")
        # break
vehicle.close()
