import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import random

def generate_launch_description():
    """Launch multiple drone control nodes for swarm operation"""
    
    ld = LaunchDescription()
    nodes = []
    occupied = [(0,0)] 
    min_dist = 2.0     
    drone_poses = []  
    for i in range(2): 
        while True:
        
            rx = random.randint(-5, 5)
            ry = random.randint(-5, 5)
            conflict = False
            for (ox, oy) in occupied:
                dist = ((rx - ox)**2 + (ry - oy)**2)**0.5
                if dist < min_dist:
                    conflict = True
                    break
            if not conflict:
                occupied.append((rx, ry))
                drone_poses.append(f"{rx},{ry}")
                break
    
    #  Define Paths to the PX4 folder
    home_dir = os.path.expanduser('~')
    px4_dir = os.path.join(home_dir, 'PX4-Autopilot')
    micro_xrce_agent_dir = os.path.join(home_dir, "Micro-XRCE-DDS-Agent")
    qgc_dir = os.path.join(home_dir, "QGC")
    
    # Starting Micro XRCE Agent
    manual_command_1 = (
        "MicroXRCEAgent ",
        "udp4 ",
        "-p ",
        "8888" 
    )
    nodes.append(ExecuteProcess(
            cmd=[manual_command_1],
            shell=True,
            cwd=micro_xrce_agent_dir,
            output='screen'
        )   
    )
    
    # Starting QGroundControl
    manual_command_2 = (
        "./QGroundControl.AppImage"
    )
    nodes.append(ExecuteProcess(
            cmd=[manual_command_2],
            shell=True,
            cwd=qgc_dir,
            output='screen'
        )   
    )
    
    # Starting gz and spwaning Drone 1
    nodes.append(ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500'],
        cwd=px4_dir,
        output='screen'
    ))
    
    # Spwaning Drone 2
    manual_command_3 = (
        "PX4_SYS_AUTOSTART=4001 "
        "PX4_SIM_MODEL=gz_x500 "
        f"PX4_GZ_MODEL_POSE='{drone_poses[0]}' "
        "./build/px4_sitl_default/bin/px4 -i 1"
    )
    nodes.append(TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[manual_command_3],
                shell=True,
                cwd=px4_dir,
                output='screen'
            )
        ]
    ))
    
    # Spwaning Drone 2
    manual_command_4 = (
        "PX4_SYS_AUTOSTART=4001 "
        "PX4_SIM_MODEL=gz_x500 "
        f"PX4_GZ_MODEL_POSE='{drone_poses[1]}' "
        "./build/px4_sitl_default/bin/px4 -i 2"
    )
    nodes.append(TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[manual_command_4],
                shell=True,
                cwd=px4_dir,
                output='screen'
            )
        ]
    ))
    
    nodes.append(TimerAction(
        period=50.0,
        actions=[
            Node(
                package='drone_control',
                executable='swarm_first_mission',
                name='swarm_first_mission',
                output='screen'
            )
        ]      
    ))
    
    for node in nodes:
        ld.add_action(node)
        
    return ld