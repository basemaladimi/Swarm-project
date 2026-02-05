import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleGlobalPosition

class SwarmFirstMission(Node):
    def __init__(self):
        super().__init__('swarm_first_mission')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publishers ---
        # Drone 1 (Leader)
        self.pub_occ_1 = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.pub_traj_1 = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.pub_cmd_1 = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Drone 2 (Right)
        self.pub_occ_2 = self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', qos_profile)
        self.pub_traj_2 = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', qos_profile)
        self.pub_cmd_2 = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', qos_profile)
        
        # Drone 3 (Left)
        self.pub_occ_3 = self.create_publisher(OffboardControlMode, '/px4_2/fmu/in/offboard_control_mode', qos_profile)
        self.pub_traj_3 = self.create_publisher(TrajectorySetpoint, '/px4_2/fmu/in/trajectory_setpoint', qos_profile)
        self.pub_cmd_3 = self.create_publisher(VehicleCommand, '/px4_2/fmu/in/vehicle_command', qos_profile)

        # --- Subscribers ---
        self.sub_global_pos_1 = self.create_subscription(
            VehicleGlobalPosition, 
            '/fmu/out/vehicle_global_position', 
            self.position_callback_msg, 
            qos_profile
        )

        self.leader_lat = None
        self.leader_lon = None
        self.leader_alt = None

        # --- 1. Background Timer (Just for Heartbeats) ---
        # Runs at 10Hz to keep Offboard mode active
        self.timer = self.create_timer(0.1, self.timer_callback)

        # --- 2. Start the Mission Thread ---
        # This runs your sequential logic separately
        self.mission_thread = threading.Thread(target=self.perform_mission)
        self.mission_thread.daemon = True # Ensures thread dies when script ends
        self.mission_thread.start()

    def position_callback_msg(self, msg):
        self.leader_lat = msg.lat
        self.leader_lon = msg.lon
        self.leader_alt = msg.alt

    def send_cmd(self, publisher, command, sys_id, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = sys_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        publisher.publish(msg)
    
    def send_reposition_cmd(self, publisher, sys_id, lat, lon, alt):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_REPOSITION
        msg.param1 = -1.0
        msg.param2 = 1.0  
        msg.param5 = lat     
        msg.param6 = lon
        msg.param7 = alt
        msg.target_system = sys_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        publisher.publish(msg)

    def timer_callback(self):
        occ = OffboardControlMode()
        occ.position, occ.velocity, occ.acceleration = True, False, False
        self.pub_occ_1.publish(occ)
        self.pub_occ_2.publish(occ)
        self.pub_occ_3.publish(occ)
        
        t1 = TrajectorySetpoint()
        t1.position = [0.0, 0.0, -5.0]
        self.pub_traj_1.publish(t1)
        self.pub_traj_2.publish(t1)
        self.pub_traj_3.publish(t1)

    def perform_mission(self):
        time.sleep(2.0)
        self.send_cmd(self.pub_cmd_1, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 1.0, 6.0)
        self.send_cmd(self.pub_cmd_2, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 2, 1.0, 6.0)
        self.send_cmd(self.pub_cmd_3, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 3, 1.0, 6.0)
        time.sleep(2.0)
        self.send_cmd(self.pub_cmd_1, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, 1.0)
        self.send_cmd(self.pub_cmd_2, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 2, 1.0)
        self.send_cmd(self.pub_cmd_3, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 3, 1.0)
        self.get_logger().info("Taking off... Waiting 10 seconds.")
        time.sleep(10.0)

        # Form the Line (Using Global Reposition)
        self.get_logger().info("Phase 3: Moving to Line Formation")
        # Wait until we actually have GPS data
        while self.leader_lat is None:
            self.get_logger().info("Waiting for Leader GPS...")
            time.sleep(1.0)

        m_to_deg = 0.00001
        
        # Drone 2: 2m North of Leader
        d2_lat = self.leader_lat + (2.0 * m_to_deg)
        d2_lon = self.leader_lon
        # Drone 3: 2m South of Leader
        d3_lat = self.leader_lat - (2.0 * m_to_deg)
        d3_lon = self.leader_lon
        current_alt = self.leader_alt
        self.send_reposition_cmd(self.pub_cmd_2, 2, d2_lat, d2_lon, current_alt)
        self.send_reposition_cmd(self.pub_cmd_3, 3, d3_lat, d3_lon, current_alt)
        time.sleep(10.0)
        #  MOVE DRONE 1 FORWARD
        self.get_logger().info("Moving Leader 1 Meter Forward")
        target_lat_1 = self.leader_lat 
        target_lon_1 = self.leader_lon - (2.0 * m_to_deg)
        self.send_reposition_cmd(self.pub_cmd_1, 1, target_lat_1, target_lon_1, current_alt)
        
        self.get_logger().info("Leader moving. Mission Complete.")

def main(args=None):
    rclpy.init(args=args)
    node = SwarmFirstMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()