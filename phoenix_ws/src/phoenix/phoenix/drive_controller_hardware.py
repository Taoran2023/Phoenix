import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import InputRc

# roboclaw and jetson
from phoenix.mpc.DriveControllerBase import DriveControllerBase
# from atmo.mpc.roboclaw_3 import Roboclaw
from dynamixel_sdk import *
from phoenix.mpc.parameters import params_

# get parameters
min                    = params_.get('min')
max                    = params_.get('max')
dead                   = params_.get('dead')
Ts                     = params_.get('Ts_drive_controller')

pitch_channel          = params_.get('pitch_channel')
roll_channel           = params_.get('roll_channel')
offboard_channel       = params_.get('offboard_channel')

# dynamixel params
drive_dynamixel_address     = params_.get('drive_dynamixel_address')
left_dxl_id                 = params_.get('left_dxl_id')
right_dxl_id                = params_.get('right_dxl_id')

torque_on_address           = params_.get('torque_on_address')
goal_velocity_address       = params_.get('goal_velocity_address') 
data = 1 # enable torque, trigger

class DriveControllerHardware(DriveControllerBase):
    def __init__(self): 
        super().__init__()

        self.subscription = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.rc_listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        # initialize drive speed and turn speed
        self.drive_speed_in = dead
        self.turn_speed_in  = dead

        # Manual vs automatic control
        self.manual = True

        # roboclaw stuff
        # self.address = 0x80
        # self.rc = Roboclaw(drive_roboclaw_address,115200)
        # self.rc.Open()

        # dynamixel config and init
        self.dxl_portHandler = PortHandler(drive_dynamixel_address)
        self.dxl_packetHandler = PacketHandler(2.0)


        if not self.dxl_portHandler.openPort():
            print("Failed to open the port!")
            exit()

        if not self.dxl_portHandler.setBaudRate(57600):
            print("Failed to change the baudrate!")
            exit()


        dxl_comm_result, dxl_error = self.dxl_packetHandler.write1ByteTxRx(self.dxl_portHandler, left_dxl_id, torque_on_address, data)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.dxl_packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.dxl_packetHandler.getRxPacketError(dxl_error))
        else:
            print("left_dxl Dynamixel has been successfully connected")

        dxl_comm_result, dxl_error = self.dxl_packetHandler.write1ByteTxRx(self.dxl_portHandler, right_dxl_id, torque_on_address, data)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.dxl_packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.dxl_packetHandler.getRxPacketError(dxl_error))
        else:
            print("right_dxl Dynamixel has been successfully connected")


    def rc_listener_callback(self, msg):
        self.drive_speed_in  = msg.values[pitch_channel]
        self.turn_speed_in   = msg.values[roll_channel]

        # set manual or automatic control of tilt angle
        if msg.values[offboard_channel] == max:
            self.manual = False
        else:
            self.manual = True

    def normalize(self,drive_speed_in):
        return (float(drive_speed_in)-float(dead))/float(max-dead)

    def map_speed(self,speed_normalized):
        # return int(127*speed_normalized)
        return int(128*speed_normalized)


# 
    def on_shutdown(self):
        self.dxl_packetHandler.write1ByteTxRx(self.dxl_portHandler, left_dxl_id, torque_on_address, 0)
        self.dxl_packetHandler.write1ByteTxRx(self.dxl_portHandler, right_dxl_id, torque_on_address, 0)
        self.dxl_portHandler.closePort()
        self.get_logger().info("port closed !")

    def move_right_wheel(self, speed):
        if abs(speed) > 128:
            speed = int(speed/abs(speed)*128)
        dxl_comm_result, dxl_error = self.dxl_packetHandler.write4ByteTxRx(self.dxl_portHandler, right_dxl_id, goal_velocity_address, speed)
        print(speed)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.dxl_packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.dxl_packetHandler.getRxPacketError(dxl_error))

    def move_left_wheel(self, speed):
        if abs(speed) > 128:
            speed = int(speed/abs(speed)*128)
        # left is reversed
        speed = -1 * speed
        self.dxl_packetHandler.write4ByteTxRx(self.dxl_portHandler, left_dxl_id, goal_velocity_address, speed)

    def update(self):
        print("self.manual: ", self.manual)
        if (self.manual):
            # manual control of driving
            # self.drive_speed_in = self.drive_speed_in + 1
            # self.turn_speed_in = self.turn_speed_in + 1
            # if self.drive_speed_in > max:
            #     self.drive_speed_in = min
            #     self.turn_speed_in = min

            lin_vel = self.map_speed(self.normalize(self.drive_speed_in))
            ang_vel = self.map_speed(self.normalize(self.turn_speed_in))

            self.get_logger().info(f"lin_vel, ang_vel: ({(self.normalize(self.drive_speed_in))},{ang_vel})")

            self.move_right_wheel(lin_vel + ang_vel)
            self.move_left_wheel(lin_vel - ang_vel)
            # self.move_right_wheel(lin_vel)
            # self.move_left_wheel(lin_vel)
        else:
            # automatic control of driving
            lin_vel = -self.map_speed(self.drive_speed)
            ang_vel = -self.map_speed(self.turn_speed)
            self.move_right_wheel(lin_vel + ang_vel)
            self.move_left_wheel(lin_vel - ang_vel)
     
def main(args=None):
    rclpy.init(args=args)
    drive_controller = DriveControllerHardware()
    drive_controller.get_logger().info("Starting DriveControllerHardware node...")
    rclpy.spin(drive_controller)
    drive_controller.on_shutdown()  # do any custom cleanup
    drive_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()