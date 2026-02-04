import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class SwarmFirstMission(Node):
    def __init__(self):
        super().__init__('swarm_first_mission')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )
        self.offboard_control_mode_publisher_1 = self.create_publisher(
            OffboardControlMode,
            '/px4_1/fmu/in/offboard_control_mode',
            qos_profile
        )

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        self.trajectory_setpoint_publisher_1 = self.create_publisher(
            TrajectorySetpoint,
            '/px4_1/fmu/in/trajectory_setpoint',
            qos_profile
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )
        self.vehicle_command_publisher_1 = self.create_publisher(
            VehicleCommand,
            '/px4_1/fmu/in/vehicle_command',
            qos_profile
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)
        self.vehicle_command_publisher_1.publish(msg)
        
    def publish_vehicle_command1(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 2
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)
        self.vehicle_command_publisher_1.publish(msg)
    
    def timer_callback(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        self.offboard_control_mode_publisher.publish(msg)
        self.offboard_control_mode_publisher_1.publish(msg)
        
        traj = TrajectorySetpoint()
        traj.position = [0.0, 0.0, -5.0]
        self.trajectory_setpoint_publisher.publish(traj)
        self.trajectory_setpoint_publisher_1.publish(traj)
        
        if self.counter ==10:
            self.get_logger().info("Switching to OFFBOARD mode")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_vehicle_command1(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if self.counter == 20:
            self.get_logger().info("Arming the drone")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.publish_vehicle_command1(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.counter += 1
        
def main(args=None):
    rclpy.init(args=args)
    node = SwarmFirstMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()