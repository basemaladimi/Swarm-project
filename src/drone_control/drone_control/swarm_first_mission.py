import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class SwarmFirstMission(Node):
    def __init__(self):
        super().__init__('swarm_first_mission')

        # --- 1. GET PARAMETERS (CRITICAL FIX) ---
        # This is the part you were missing. We read the offsets passed from the launch file.
        self.declare_parameter('offset_d2', [0.0, 0.0])
        self.declare_parameter('offset_d3', [0.0, 0.0])
        
        # We store them in 'self.off_2' and 'self.off_3' so the timer can see them.
        self.off_2 = self.get_parameter('offset_d2').value
        self.off_3 = self.get_parameter('offset_d3').value
        
        self.get_logger().info(f"offsets received: D2={self.off_2}, D3={self.off_3}")

        # --- 2. SETUP PUBLISHERS ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

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

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0

    def send_cmd(self, publisher, command, sys_id, param1=0.0, param2=0.0):
        """Helper to send vehicle commands like Arm or Change Mode"""
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

    def timer_callback(self):
        # --- 1. Heartbeat ---
        occ = OffboardControlMode()
        occ.position, occ.velocity, occ.acceleration = True, False, False
        self.pub_occ_1.publish(occ)
        self.pub_occ_2.publish(occ)
        self.pub_occ_3.publish(occ)

        # --- 2. Formation Logic ---
        target_z = -5.0 # Height

        # Drone 1 (Leader) -> Stay at 0,0
        t1 = TrajectorySetpoint()
        t1.position = [0.0, 0.0, target_z]
        self.pub_traj_1.publish(t1)
        
        # Drone 2 (Right) -> Go to (0, 2)
        # MATH: Target - Spawn_Offset
        t2 = TrajectorySetpoint()
        t2.position = [
            0.0 - self.off_2[0],  
            2.0 - self.off_2[1],  
            target_z
        ]
        self.pub_traj_2.publish(t2)
        
        # Drone 3 (Left) -> Go to (0, -2)
        # MATH: Target - Spawn_Offset
        t3 = TrajectorySetpoint()
        t3.position = [
            0.0 - self.off_3[0],  
            -2.0 - self.off_3[1], 
            target_z
        ]
        self.pub_traj_3.publish(t3)

        # --- 3. Sequence Logic ---
        if self.counter == 10:
            self.get_logger().info("Phase 1: OFFBOARD Mode")
            self.send_cmd(self.pub_cmd_1, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 1.0, 6.0)
            self.send_cmd(self.pub_cmd_2, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 2, 1.0, 6.0)
            self.send_cmd(self.pub_cmd_3, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 3, 1.0, 6.0)

        elif self.counter == 20:
            self.get_logger().info("Phase 2: Arming & Takeoff")
            self.send_cmd(self.pub_cmd_1, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, 1.0)
            self.send_cmd(self.pub_cmd_2, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 2, 1.0)
            self.send_cmd(self.pub_cmd_3, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 3, 1.0)
            
        elif self.counter == 100: 
            self.get_logger().info("Phase 3: Waiting in Formation...")

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SwarmFirstMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()