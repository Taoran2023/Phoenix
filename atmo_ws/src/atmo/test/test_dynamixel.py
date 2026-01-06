#!/usr/bin/env python3

from dynamixel_sdk import *


portHandler = PortHandler("/dev/ttyUSB0")
packetHandler = PacketHandler(2.0)

if portHandler.openPort():
  print("Succeeded to open the port!")
else:
  print("Failed to open the port!")
  exit()

if portHandler.setBaudRate(57600):
  print("Succeeded to change the baudrate!")
else:
  print("Failed to change the baudrate!")
  exit()

dxl_id = 1
torque_on_address = 64
data = 1
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, data)

if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

while True:
    try:
        target_position = int(input("Enter target position (0 ~ 4095, -1 to exit): "))
    except ValueError:
        print("Please enter an integer.")
        continue

    if target_position == -1:
        break
    elif target_position < 0 or target_position > 4095:
        print("Position must be between 0 and 4095.")
        continue

    # goal_position_address = 116
    goal_position_address = 104  # Goal Velo

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, goal_position_address, target_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    while True:
        present_position_address = 132
        present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, present_position_address)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print(f"Current Position: {present_position}")

        if abs(target_position - present_position) <= 10:
            break

data = 0
packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, data)
portHandler.closePort()