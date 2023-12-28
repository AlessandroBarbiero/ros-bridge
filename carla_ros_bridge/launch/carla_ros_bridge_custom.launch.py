import launch
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import json
from transforms3d.euler import euler2quat
import math

CONFIG_DIR = r'/home/ale/carla-ros-bridge/src/ros-bridge/carla_ros_bridge/config'

def carla_rotation_to_RPY(roll, pitch, yaw):
    """
    Convert a carla rotation to a roll, pitch, yaw tuple

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    """
    _roll = math.radians(roll)
    _pitch = -math.radians(pitch)
    _yaw = -math.radians(yaw)

    return (_roll, _pitch, _yaw)

def carla_rotation_to_quat(roll, pitch, yaw):
    _roll, _pitch, _yaw = carla_rotation_to_RPY(roll=roll, pitch=pitch, yaw=yaw)
    quat = euler2quat(_roll, _pitch, _yaw)
    return quat

def generate_launch_description():

    static_tf_nodes = []

    f = open(CONFIG_DIR + '/sensors_homes.json')
    sensors_config = json.load(f)
    sensors_home_actors = sensors_config["actors"]

    for sh in sensors_home_actors:
        location = sh["location"]
        x = str(location["x"])
        y = str(-location["y"]) # Invert y because we are passing from left to right hand system
        z = str(location["z"])
        rotation = sh["rotation"]
        quat = carla_rotation_to_quat(**rotation)
        qw = str(quat[0])
        qx = str(quat[1])
        qy = str(quat[2])
        qz = str(quat[3])

        static_tf_nodes.append(
            # Node(
            #     package='tf2_ros',
            #     executable='static_transform_publisher',
            #     arguments = ['2.6', '21.95', '0.15', '0', '0', '0.99688761', '0.07883583', 'map', 'sensors_home']
            # )

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments = [x,y,z,   qx, qy, qz, qw, 'map', sh["name"]]
            )
        )
    
    launch_args = [
        launch.actions.DeclareLaunchArgument(
            name='host',
            # default_value='localhost',
            default_value='192.168.1.1',
            description='IP of the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000',
            description='TCP port of the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='100',
            description='Time to wait for a successful connection to the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='passive',
            default_value='False',
            description='When enabled, the ROS bridge will take a backseat and another client must tick the world (only in synchronous mode)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode',
            default_value='True',
            description='Enable/disable synchronous mode. If enabled, the ROS bridge waits until the expected data is received for all sensors'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False',
            description='When enabled, pauses the tick until a vehicle control is completed (only in synchronous mode)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.017', # To avoid aliasing I set frequency to double the one of the slowest sensor (~60fps)
            description='Simulation time (delta seconds) between simulation steps'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='TownBovisa/Maps/TownBovisa/TownBovisa',
            description='Either use an available CARLA town (eg. "Town01") or an OpenDRIVE file (ending in .xodr)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='register_all_sensors',
            default_value='True',
            description='Enable/disable the registration of all sensors. If disabled, only sensors spawned by the bridge are registered'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ego_vehicle_role_name',
            default_value=["hero", "ego_vehicle", "hero0", "hero1", "hero2",
                           "hero3", "hero4", "hero5", "hero6", "hero7", "hero8", "hero9"],
            description='Role names to identify ego vehicles. '
        )
    ]

    all_args = launch_args + static_tf_nodes + [
        Node(
            package='carla_ros_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': True
                },
                {
                    'host': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'port': launch.substitutions.LaunchConfiguration('port')
                },
                {
                    'timeout': launch.substitutions.LaunchConfiguration('timeout')
                },
                {
                    'passive': launch.substitutions.LaunchConfiguration('passive')
                },
                {
                    'synchronous_mode': launch.substitutions.LaunchConfiguration('synchronous_mode')
                },
                {
                    'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command')
                },
                {
                    'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
                },
                {
                    'town': launch.substitutions.LaunchConfiguration('town')
                },
                {
                    'register_all_sensors': launch.substitutions.LaunchConfiguration('register_all_sensors')
                },
                {
                    'ego_vehicle_role_name': launch.substitutions.LaunchConfiguration('ego_vehicle_role_name')
                }
            ]
        ),
        TimerAction(
            period=2.0, #Add 2 seconds delay to register static objects at the beginning
            actions=[
                Node(
                    package='carla_spawn_objects',
                    executable='carla_spawn_objects',
                    name='carla_spawn_objects',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        {
                            'objects_definition_file': '/home/ale/carla-ros-bridge/src/ros-bridge/carla_ros_bridge/config/objects_to_spawn.json'
                        },
                        {
                            'spawn_sensors_only': True
                        }
                    ]
                ),
            ]
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', 'multi_sens'],
        )
    ]


    ld = launch.LaunchDescription(all_args)
    return ld


if __name__ == '__main__':
    generate_launch_description()
