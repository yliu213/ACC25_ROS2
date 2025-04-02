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

    # Make sure PX4 SITL dirs exist
    os.makedirs(os.path.join(PX4_RUN_DIR, "sitl_uav0"), exist_ok=True)
    os.makedirs(os.path.join(PX4_RUN_DIR, "sitl_uav1"), exist_ok=True)

    # Gazebo Plugin Path
    gazebo_plugin_path = os.path.join(px4_build_dir, "build_gazebo-classic")

    # Package
    dsls_share_dir = get_package_share_directory('dsls_dea')

    # World
    world_path = os.path.join(HOME, 'PX4-Autopilot','Tools','simulation','gazebo-classic','sitl_gazebo-classic','worlds','empty.world')

    # Models
    load_model_path = os.path.join(dsls_share_dir, 'models','double_slung_load_point_mass','double_slung_load_point_mass.sdf')
    uav_model_path = os.path.join(dsls_share_dir, 'models','px4vision_ancl','px4vision_ancl.sdf')

    # Env
    env_vars = [
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(HOME, 'PX4-Autopilot','Tools','simulation','gazebo-classic','sitl_gazebo-classic','models') + ':' +
            os.path.join(dsls_share_dir, 'models')
        ),
    ]

    # Launch arguments
    world_arg = DeclareLaunchArgument('world', default_value=world_path)
    uav_model_arg = DeclareLaunchArgument('model', default_value=uav_model_path)
    dsl_sdf_arg = DeclareLaunchArgument('dsl_sdf', default_value=load_model_path)

    # Gazebo launch
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'), 'verbose': 'true', 'pause': 'true'}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py'))
    )

    # ------------------------------------------------------------------
    # UAV0
    spawn_uav0 = ExecuteProcess(
        cmd=["gz", "model", "--spawn-file", LaunchConfiguration('model'), "--model-name", "px4vision_0", "-x", "0.0", "-y", "0.85", "-z", "0.0"],
        output='screen',
        prefix="bash -c 'sleep 5; $0 $@'"
    )

    px4_sitl_uav0 = ExecuteProcess(
        cmd=[
            os.path.join(px4_build_dir, 'bin', 'px4'),
            os.path.join(HOME, "PX4-Autopilot","ROMFS","px4fmu_common"),
            '-i', '0', # must be unique for each UAV
            '-s', os.path.join(HOME, "PX4-Autopilot","ROMFS","px4fmu_common","init.d-posix","rcS")
        ],
        cwd=os.path.join(PX4_RUN_DIR, "sitl_uav0"), # different dir for each UAV
        shell=True,
        prefix="xterm -hold -e",
        output='screen'
    )

    # Agent for UAV0 (2019 is default PX4 convention)
    # = uav_instance_id + dds_port
    agent0 = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '2019'],
        output='screen'
    )

    # ---------------------------
    # UAV1
    spawn_uav1 = ExecuteProcess(
        cmd=["gz", "model", "--spawn-file", LaunchConfiguration('model'), "--model-name", "px4vision_1", "-x", "0.0", "-y", "-0.85", "-z", "0.0"],
        output='screen',
        prefix="bash -c 'sleep 5; $0 $@'"
    )

    px4_sitl_uav1 = ExecuteProcess(
        cmd=[
            os.path.join(px4_build_dir, 'bin', 'px4'),
            os.path.join(HOME, "PX4-Autopilot","ROMFS","px4fmu_common"),
            '-i', '1',
            '-s', os.path.join(HOME, "PX4-Autopilot","ROMFS","px4fmu_common","init.d-posix","rcS")
        ],
        cwd=os.path.join(PX4_RUN_DIR, "sitl_uav1"),
        shell=True,
        prefix="xterm -hold -e",
        output='screen'
    )

    # Agent for UAV1
    agent1 = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '2020'],
        output='screen'
    )

    # ---------------------------
    # Slung Load
    spawn_load = ExecuteProcess(
        cmd=["gz", "model", "--spawn-file", LaunchConfiguration('dsl_sdf'), "--model-name", "slung_load"],
        output='screen',
        prefix="bash -c 'sleep 5; $0 $@'"
    )

    return LaunchDescription([
        *env_vars,

        world_arg,
        dsl_sdf_arg,
        uav_model_arg,

        gzserver,
        gzclient,

        spawn_uav0,
        px4_sitl_uav0,
        agent0,

        spawn_uav1,
        px4_sitl_uav1,
        agent1,
        
        spawn_load,
    ])
