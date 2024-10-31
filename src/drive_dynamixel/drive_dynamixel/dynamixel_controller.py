import rclpy
from rclpy.node import Node
import math
from dynamixel_sdk import *
from result_msgs.msg import Force
from result_msgs.msg import Person
from result_msgs.msg import Mode

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')

        
        self.force_info_sub_ = self.create_subscription(
            Force,
            '/force_info',
            self.force_info_callback,
            10)
        
        self.person_info_sub_ = self.create_subscription(
            Person,
            '/person_info',
            self.person_info_callback,
            10)
        
        self.mode_info_sub_ = self.create_subscription(
            Mode,
            '/mode_info',
            self.mode_info_callback,
            10)
        
        self.mode = 127
        self.DEBUG = False

        # Control table address
        self.ADDR_MX_TORQUE_ENABLE       = 64                              # Control table address is different in Dynamixel model
        self.ADDR_MX_LED_ENABLE          = 65                            
        self.ADDR_MX_GOAL_POSITION       = 116
        self.ADDR_MX_PRESENT_POSITION    = 132
        self.ADDR_PROFILE_ACCELERATION   = 108
        self.ADDR_PROFILE_VELOCITY       = 112
        self.ADDR_MOVING                 = 122
        self.ADDR_POSITION_I_GAIN        = 82
        self.ADDR_POSITION_P_GAIN        = 84

        # Protocol version
        self.PROTOCOL_VERSION            = 1                               # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID                      = [0, 1, 2, 3]                          # Dynamixel ID list
        self.BAUDRATE                    = 115200
        self.DEVICENAME                  = "/dev/ttyUSB0"                  # Check which port is being used on your controller
                                                                           # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

        self.TORQUE_ENABLE               = 1                               # Value for enabling the torque
        self.LED_ENABLE                  = 1                               # Value for enabling the LED
        self.PROFILE_ACCELERATION        = 20
        self.PROFILE_VELOCITY            = 150
        self.TORQUE_DISABLE              = 0                               # Value for disabling the torque
        self.LED_DISABLE                 = 0                               # Value for enabling the LED
        self.DXL_MOVING_STATUS_THRESHOLD = 20                              # Dynamixel moving status threshold
        self.COMM_SUCCESS                = 0                               # Communication Success result value
        self.LEN_MX_GOAL_POSITION        = 4
        self.LEN_MX_PRESENT_POSITION     = 4
        self.OFFSET                      = [-2048+2129, 
                                            -2048+2235, 
                                            -2048+2146, 
                                            -2048+1718]

        self.portHandler_ = PortHandler(self.DEVICENAME)
        self.portHandler_.setBaudRate(self.BAUDRATE)
        self.packetHandler_ = PacketHandler(self.PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler_, 
                                             self.packetHandler_, 
                                             self.ADDR_MX_GOAL_POSITION, 
                                             self.LEN_MX_GOAL_POSITION)
        self.groupBulkRead = GroupBulkRead(self.portHandler_, 
                                           self.packetHandler_)

        self.dxl_goal_position      = [0 for _ in range(len(self.DXL_ID))]     # Goal position
        self.dxl_present_position   = [0 for _ in range(len(self.DXL_ID))]     # Present position
        self.dxl_moving             = [0 for _ in range(len(self.DXL_ID))]     # Moving

        self.pre_angle = 90.0
        self.pre_length = 1000
        self.pre_degree = 90.0
        self.non_target_cnt = 0
        
        # Open port
        if not (self.portHandler_.openPort()):
            self.get_logger().info("포트를 열 수 없습니다!")
        
        # Init
        self.set_dxl_torque(self.TORQUE_ENABLE)
        self.set_dxl_led(self.LED_ENABLE)
        self.set_dxl_profile(self.PROFILE_ACCELERATION, self.PROFILE_VELOCITY)
        self.set_dxl_position_gain(self.ADDR_POSITION_I_GAIN, 100, self.ADDR_POSITION_P_GAIN, 700)
        self.set_multi_goal_position([2048, 2048, 2048, 2048])
        time.sleep(3.0)

        # Test
        # self.test_sync_bulk()

    def mode_info_callback(self, msg):
        self.mode = msg.mode
        
    def force_info_callback(self, msg):
        
        if self.mode == 4:
            value = math.sqrt(msg.x ** 2 + msg.y ** 2)
            
            if value < 500.0:
                return
            
            if msg.x == 0.0 and msg.y == 0:
                angle = 90.0
            else:         
                angle = self.rad2deg(math.atan2(msg.y, msg.x))
            
            if angle < 0:
                angle += 180.0
            
            """           
            if abs(angle - self.pre_angle) > 150.0:
                angle = 180 - angle
            
            self.pre_angle = angle
            """
            
            self.get_logger().info(f"\n\nmsg: {msg}\nforce deg: {angle}\nvector: {value}\nmode: {self.mode}\n")
            
            goal_positions = [0, 0, 0, 0]
                        
            goal_positions[0] = 4095 - (int(angle / 180.0 * 2048) + 1024) # FL
            goal_positions[1] = int(angle / 180.0 * 2048) + 1024 # BL
            goal_positions[2] = int(angle / 180.0 * 2048) + 1024 # BR
            goal_positions[3] = 4095 - (int(angle / 180.0 * 2048) + 1024) # FR
            
            if 75.0 <= angle <= 115.0:
                goal_positions[0] = 2048
                goal_positions[1] = 2048
                goal_positions[2] = 2048
                goal_positions[3] = 2048
            
            if 0.0 <= angle <= 15.0 or 165.0 <= angle <= 180.0:
                goal_positions[0] = 3072
                goal_positions[1] = 3072
                goal_positions[2] = 1024
                goal_positions[3] = 1024
                
            """
            if 0.0 < angle < 90.0:
                goal_positions[1] = 4096 - goal_positions[1]
                goal_positions[3] = 4096 - goal_positions[3]
            """
                
            
            for id in self.DXL_ID:
                if goal_positions[id] < 1024:
                    goal_positions[id] = 1024
                elif goal_positions[id] > 3072:
                    goal_positions[id] = 3072
            
            self.get_logger().info(f"goal positions: {goal_positions}\n")
            
            self.set_multi_goal_position(goal_positions)
        
        
        if self.mode == 6:
            value = math.sqrt(msg.x ** 2 + msg.y ** 2)
            if value < 500.0:
                return
            
            if msg.x == 0.0 and msg.y == 0:
                angle = 90.0
            else:         
                angle = self.rad2deg(math.atan2(msg.y, msg.x))
            
            if angle < 0:
                angle += 180.0
            
            goal_positions = [0, 0, 0, 0]
            
            if 0.0 <= angle <= 22.5:
                goal_positions[0] = 3072
                goal_positions[1] = 3072
                goal_positions[2] = 1024
                goal_positions[3] = 1024
            elif 22.5 < angle <= 56.25:
                goal_positions[0] = 2560
                goal_positions[1] = 1536
                goal_positions[2] = 1536
                goal_positions[3] = 2560
            elif 56.25 < angle <= 78.75:
                goal_positions[0] = 2304
                goal_positions[1] = 1792
                goal_positions[2] = 1792
                goal_positions[3] = 2304
            elif 78.75 < angle <= 101.25:
                goal_positions[0] = 2048
                goal_positions[1] = 2048
                goal_positions[2] = 2048
                goal_positions[3] = 2048
            elif 101.25 < angle <= 123.75:
                goal_positions[0] = 1792
                goal_positions[1] = 2304
                goal_positions[2] = 2304
                goal_positions[3] = 1792
            elif 123.75 < angle <= 157.5:
                goal_positions[0] = 1536
                goal_positions[1] = 2560
                goal_positions[2] = 2560
                goal_positions[3] = 1536
            elif 157.5 < angle <= 180.0:
                goal_positions[0] = 3072
                goal_positions[1] = 3072
                goal_positions[2] = 1024
                goal_positions[3] = 1024
            
                
            self.get_logger().info(f"\n\nmsg: {msg}\nforce deg: {angle}\nvector: {value}\n")
            
            for id in self.DXL_ID:
                if goal_positions[id] < 1024:
                    goal_positions[id] = 1024
                elif goal_positions[id] > 3072:
                    goal_positions[id] = 3072
            
            self.get_logger().debug(f"mode: {self.mode}) goal positions: {goal_positions}")
            
            self.set_multi_goal_position(goal_positions)
        
                
    def person_info_callback(self, msg):
        
        if self.mode == 5:
            if msg.flag == 2:
                self.set_multi_goal_position([2048, 2048, 2048, 2048])
                self.get_logger().info(f"Not Received UDP Communication!! \n FLAG: {msg.flag}")
                return
                  
            self.get_logger().debug("person info callback")
    
            if not(0.3 <= msg.length <= 3.0):
                msg.length = 1.0

            if not(60.0 <= msg.degree <= 120.0):
                msg.degree = 90.0
            
            self.get_logger().debug(f"before) length: {msg.length}, degree: {msg.degree}")
            length_factor = 1000.0
            length = msg.length * length_factor
            degree_factor = 1.0
            degree = (msg.degree - 90.0) * degree_factor
        
            if msg.flag == 1:
                self.non_target_cnt = 0
                self.pre_length = length
                self.pre_degree = degree
            else:
                self.non_target_cnt += 1
                length = self.pre_length
                degree = self.pre_degree
            
            if self.non_target_cnt > 20:
                self.non_target_cnt = 0
                length = 1000
                
                if self.pre_degree < 0:
                    degree = -30.0
                else:
                    degree = 30.0
            
            
            self.get_logger().info(f"length: {length:.3f}")
            self.get_logger().info(f"degree: {degree:.3f}")
            self.get_logger().info(f"FALG: {msg.flag}")
            self.get_logger().info(f"non target cnt: {self.non_target_cnt}")
            
            steering_info = self.steering_kinematics(length, degree)
        
            goal_positions = []
            for radian in steering_info[:-1]:
                if math.isnan(math.fabs(self.rad2deg(radian))):
                    goal_positions.append(2048)
                else:
                    goal_positions.append(int(math.fabs(self.rad2deg(radian) / 90.0 * 2048)))

            if degree < 0.0:
                for i in range(4):
                    goal_positions[i] = 4096 - goal_positions[i]

            self.get_logger().debug(f"FL: {goal_positions[0]}")
            self.get_logger().debug(f"FR: {goal_positions[1]}")
            self.get_logger().debug(f"BL: {goal_positions[2]}")
            self.get_logger().debug(f"BR: {goal_positions[3]}")

            self.set_multi_goal_position([goal_positions[0], goal_positions[2], goal_positions[3], goal_positions[1]])

    def set_dxl_torque(self, signal):
        for id in self.DXL_ID:
            dxl_comm_result, dxl_error = self.packetHandler_.write1ByteTxRx(self.portHandler_, id, self.ADDR_MX_TORQUE_ENABLE, signal)
            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().debug(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().debug(f"{self.packetHandler_.getRxPacketError(dxl_error)}")
            else:
                if signal == self.TORQUE_ENABLE:
                    self.get_logger().debug(f"Dynamixel#{id} has been successfully torque enable")
                else:
                    self.get_logger().debug(f"Dynamixel#{id} has been successfully torque disable")

    def set_dxl_led(self, signal):
        for id in self.DXL_ID:
            dxl_comm_result, dxl_error = self.packetHandler_.write1ByteTxRx(self.portHandler_, id, self.ADDR_MX_LED_ENABLE, signal)
            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().debug(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().debug(f"{self.packetHandler_.getRxPacketError(dxl_error)}")
            else:
                if signal == self.LED_ENABLE:
                    self.get_logger().debug(f"Dynamixel#{id} has been successfully led enable")
                else:
                    self.get_logger().debug(f"Dynamixel#{id} has been successfully led disable")

    def set_dxl_profile(self, acc, vel):
        for id in self.DXL_ID:
            dxl_comm_result, dxl_error = self.packetHandler_.write4ByteTxRx(self.portHandler_, id, self.ADDR_PROFILE_ACCELERATION, acc)

            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().debug(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().debug(f"{self.packetHandler_.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().debug(f"Dynamixel#{id} has been successfully profile acceleration: {acc}")

            dxl_comm_result, dxl_error = self.packetHandler_.write4ByteTxRx(self.portHandler_, id, self.ADDR_PROFILE_VELOCITY, vel)

            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().debug(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().debug(f"{self.packetHandler_.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().debug(f"Dynamixel#{id} has been successfully profile velocity: {vel}")

    def set_dxl_position_gain(self, addr_position_i_gain, i_gain, addr_position_p_gain, p_gain):
        for id in self.DXL_ID:
            dxl_comm_result, dxl_error = self.packetHandler_.write4ByteTxRx(self.portHandler_, id, addr_position_i_gain, i_gain)

            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().debug(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().debug(f"{self.packetHandler_.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().debug(f"Dynamixel#{id} has been successfully Position I Gain: {i_gain}")

            dxl_comm_result, dxl_error = self.packetHandler_.write4ByteTxRx(self.portHandler_, id, addr_position_p_gain, p_gain)

            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().debug(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().debug(f"{self.packetHandler_.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().debug(f"Dynamixel#{id} has been successfully Position P Gain: {p_gain}")
    
    def set_goal_position(self, id, change_goal_position):
        start_time = time.time()

        # Write goal position
        result, error = self.packetHandler_.write4ByteTxRx(self.portHandler_, id, self.ADDR_MX_GOAL_POSITION, change_goal_position)
        if result == self.COMM_SUCCESS:
            self.get_logger().debug("successfully goal position")
        else:
            self.get_logger().debug(f"Fail goal position, Error: {self.packetHandler_.getRxPacketError(error)}")

        while 1:
            # Read present position
            self.dxl_present_position = self.packetHandler_.read2ByteTxRx(self.portHandler_, id, self.ADDR_MX_PRESENT_POSITION)
            if self.dxl_present_position[0] == 65535: self.dxl_present_position[0] = 0

            # self.get_logger().info(f"[ID:{id}] GoalPos:{change_goal_position}  PresPos:{self.dxl_present_position[0]}")

            position_error = abs(change_goal_position - self.dxl_present_position[0])
            # self.get_logger().info(f"position_error: {position_error}")

            if position_error <= self.DXL_MOVING_STATUS_THRESHOLD:
                break
        end_time = time.time()
        time_taken = end_time - start_time
        self.get_logger().debug(f"successfully goal position: {change_goal_position}, time_take: {time_taken:.5f} [s]")

    def set_multi_goal_position(self, goal_postions):
        # Allocate goal position value into byte array
        for id in self.DXL_ID:
            self.get_logger().debug(f'ID: {id}, goal_postions: {goal_postions[id]}')
            self.dxl_goal_position[id] = goal_postions[id] + self.OFFSET[id]

            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(self.dxl_goal_position[id])), 
                                   DXL_HIBYTE(DXL_LOWORD(self.dxl_goal_position[id])), 
                                   DXL_LOBYTE(DXL_HIWORD(self.dxl_goal_position[id])), 
                                   DXL_HIBYTE(DXL_HIWORD(self.dxl_goal_position[id]))]

            # Add Dynamixel#ID goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(self.DXL_ID[id], param_goal_position)
            if dxl_addparam_result != True:
                self.get_logger().warning(f"[ID:{self.DXL_ID[id]:03d}] groupSyncWrite addparam failed")

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != self.COMM_SUCCESS:
            self.get_logger().warning(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

        while True:
            self.read_multi_moving()
            
            if self.dxl_moving[0] == 0 and \
               self.dxl_moving[1] == 0 and \
               self.dxl_moving[2] == 0 and \
               self.dxl_moving[3] == 0:
                   break
    
    def read_multi_moving(self):
        for id in self.DXL_ID:
            dxl_addparam_result = self.groupBulkRead.addParam(id, self.ADDR_MOVING, 1)
            if dxl_addparam_result != True:
                self.get_logger().debug(f"[ID:{id:03d}] groupBulkRead addparam failed")

            dxl_comm_result = self.groupBulkRead.txRxPacket()
            if dxl_comm_result != self.COMM_SUCCESS:
                self.get_logger().debug(f"{self.packetHandler_.getTxRxResult(dxl_comm_result)}")

            dxl_getdata_result = self.groupBulkRead.isAvailable(id, self.ADDR_MOVING, 1)
            if dxl_getdata_result != True:
                self.get_logger().debug(f"[ID:{id:03d}] groupBulkRead getdata failed")

            self.dxl_moving[id] = self.groupBulkRead.getData(id, self.ADDR_MOVING, 1)

        self.groupBulkRead.clearParam()

    def test_sync_bulk(self):
        max_err = 0
        add_goal_postion = [0, 0, 0, 0]

        while True:
            self.set_multi_goal_position(add_goal_postion)

            while True:
                self.read_multi_present_position()

                self.get_logger().debug(f"--- DXL goal position: {self.dxl_goal_position} ---")
                self.get_logger().debug(f"--- DXL pres position: {self.dxl_present_position} ---")
                self.get_logger().debug(f"--- DXL moving: {self.dxl_moving} ---")
                    
                self.check_moving_pram()
                
                if self.dxl_moving[0] == 0 and self.dxl_moving[1] == 0 and self.dxl_moving[2] == 0 and self.dxl_moving[3] == 0:
                    break

            for id in self.DXL_ID:
                add_goal_postion[id] += 100
            
            while True:
                cnt = 0
                
                for id in self.DXL_ID:
                    if add_goal_postion[id] >= 4095:
                        cnt += 1
                        add_goal_postion[id] += 100

                if cnt == 0: break
        
    def deg2rad(self, deg):
        return math.pi*deg/180.0

    def rad2deg(self, rad):
        return 180.0*rad/math.pi

    def steering_kinematics(self, dist_cam, angle_cam):   ### camera data to steer angle calculation   
        L = 490
        T = 490
        L_shift = 190
        
        ### camera to robot center transform
        dx = (L_shift) + dist_cam*math.cos(self.deg2rad(angle_cam))
        dy = dist_cam*math.sin(self.deg2rad(angle_cam))
        
        ### center to object distance and angle
        dist_cen = math.sqrt(dx**2 + dy**2)
        angle_cen = self.rad2deg(math.atan2(dy, dx))
        
        if (angle_cen == 0.0):
            # rot_radius = dist_cen/(0.000000000000001)
            rot_radius = dist_cen/(1e-15) 
        else:
            rot_radius = dist_cen/(2.0*math.sin(self.deg2rad(angle_cen)))
        
        bicycle_front = math.atan2(L/2.0, rot_radius)
        bicycle_back = -bicycle_front
        
        front_left = math.atan2(L/2.0, rot_radius - T/2.0) - math.pi/2.0
        front_right = math.atan2(L/2.0, rot_radius + T/2.0) - math.pi/2.0
        back_left = -front_left
        back_right = -front_right
        center_radius = rot_radius
        
        steering_info = [front_left, front_right, back_left, back_right, center_radius]
        
        return steering_info

    def __del__(self):
        self.set_dxl_torque(self.TORQUE_DISABLE)
        self.set_dxl_led(self.LED_DISABLE)
        self.portHandler_.closePort()

def main(args=None):
    rclpy.init(args=args)
    dynamixel_controller = DynamixelController()
    rclpy.spin(dynamixel_controller)
    dynamixel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
