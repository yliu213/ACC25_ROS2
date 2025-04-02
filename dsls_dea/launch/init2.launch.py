import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    HOME = os.environ.get("HOME", "/home/user")
    PX4_RUN_DIR = os.path.join(HOME, 'tmp/px4_run_dir')
    px4_build_dir = os.path.join(HOME, "PX4-Autopilot", "build", "px4_sitl_default")
    
    # Where Gazebo plugins are compiled
    gazebo_plugin_path = os.path.join(px4_build_dir, "build_gazebo-classic")  # For PX4 >= v1.15.0

    # You might store your DSLS models in this package’s `models` folder.
    dsls_share_dir = get_package_share_directory('dsls_dea')
    world_path = os.path.join(HOME, 'PX4-Autopilot','Tools','simulation','gazebo-classic','sitl_gazebo-classic','worlds','empty.world')

    # The model sdfs
    load_model_path = os.path.join(dsls_share_dir, 'models',
                                   'double_slung_load_point_mass',
                                   'double_slung_load_point_mass.sdf')
    uav_model_path = os.path.join(dsls_share_dir, 'models',
                                   'px4vision_dsls',
                                   'px4vision_dsls.sdf')

    # Create environment variables for Gazebo
    env_vars = [
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(HOME, 'PX4-Autopilot','Tools','simulation','gazebo-classic','sitl_gazebo-classic','models')
            + ':' + os.path.join(dsls_share_dir, 'models')
        ),
    ]

    # Launch arguments
    world_arg = DeclareLaunchArgument('world', default_value=world_path, description='Path to the world file')
    uav_model_arg = DeclareLaunchArgument('model', default_value=uav_model_path, description='Path to the UAV model')

    # standard gazebo_ros “gzserver + gzclient” launch
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true',
            'pause': 'true'  # Start paused
        }.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # --------------------------------------------------------------------
    # Now spawn the UAV0 model in Gazebo 
    spawn_uav0 = ExecuteProcess(
        cmd=[
            "gz", "model",
            "--spawn-file",  LaunchConfiguration('model'),  
            "--model-name", "px4vision_0",
            "-x", "0.0",
            "-y", "0.85",
            "-z", "0.0",
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen',
        # Delay a bit so gzserver is ready
        prefix="bash -c 'sleep 5; $0 $@'"
    )

    px4_sitl_uav0 = ExecuteProcess(
         cmd=[
                HOME + '/PX4-Autopilot/build/px4_sitl_default/bin/px4',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/',
                '-s', HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS',
            ],
            cwd=PX4_RUN_DIR,
            shell=True,
            prefix="xterm -hold -e",
            output='screen'
    )

    # --------------------------------------------------------------------
    # 2) UAV1 SITL + spawn
    # --------------------------------------------------------------------
    spawn_uav1 = ExecuteProcess(
        cmd=[
            "gz", "model",
            "--spawn-file", LaunchConfiguration('model'), 
            "--model-name", "px4vision_1",
            "-x", "0.0",
            "-y", "-0.85",
            "-z", "0.0",
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen',
        prefix="bash -c 'sleep 5; $0 $@'"
    )

    # px4_sitl_uav1 = ExecuteProcess(
    #      cmd=[
    #             HOME + '/PX4-Autopilot/build/px4_sitl_default/bin/px4',
    #             HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/',
    #             '-s', HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS',
    #         ],
    #         cwd=PX4_RUN_DIR,
    #         shell=True,
    #         prefix="xterm -hold -e",
    #         output='screen'
    # )

    # --------------------------------------------------------------------
    # 3) Spawn the double slung load SDF
    # --------------------------------------------------------------------
    spawn_load = ExecuteProcess(
        cmd=[
            "gz", "model",
            "--spawn-file", LaunchConfiguration('dsl_sdf'),  # We'll pass this as a launch arg
            "--model-name", "slung_load"
        ],
        output='screen',
        prefix="bash -c 'sleep 5; $0 $@'"
    )

    # But we need to declare the argument "dsl_sdf" first:
    dsl_sdf_arg = DeclareLaunchArgument(
        'dsl_sdf', default_value=load_model_path,
        description='Path to double slung load SDF'
    )

    # --------------------------------------------------------------------
    # 4) Link attacher
    # --------------------------------------------------------------------
    # link_attacher = ExecuteProcess(
    #     cmd=["ros2", "run", "gazebo_ros_link_attacher", "attach_sls"],
    #     output="screen",
    #     prefix="bash -c 'sleep 7; $0 $@'"
    # )

    # Start Micro XRCE-DDS Agent for ROS2 <--> PX4 communication
    xrce_dds_agent = ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen'
    )

    # Compose everything into LaunchDescription
    return LaunchDescription([
        # environment
        *env_vars,

        # arguments
        world_arg,
        dsl_sdf_arg,
        uav_model_arg,

        # Gazebo server + client
        gzserver,
        gzclient,

        # SITLs
        px4_sitl_uav0,
        #px4_sitl_uav1,

        # spawns
        spawn_uav0,
        spawn_uav1,
        spawn_load,

        # link_attacher
        # link_attacher

        # Micro XRCE-DDS Agent
        xrce_dds_agent,
    ])
