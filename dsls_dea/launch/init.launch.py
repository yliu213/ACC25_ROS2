import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    HOME = os.environ.get("HOME", "/home/user")
    PX4_RUN_DIR = os.path.join(HOME, 'tmp/px4_run_dir')
    px4_build_dir = os.path.join(HOME, "PX4-Autopilot", "build", "px4_sitl_default")

    # os.makedirs(os.path.join(PX4_RUN_DIR, "sitl_uav0"), exist_ok=True)
    # os.makedirs(os.path.join(PX4_RUN_DIR, "sitl_uav1"), exist_ok=True)

    gazebo_plugin_path = os.path.join(px4_build_dir, "build_gazebo-classic")
    dsls_share_dir = get_package_share_directory('dsls_dea')

    world_path = os.path.join(
        HOME, 'PX4-Autopilot', 'Tools', 'simulation', 'gazebo-classic',
        'sitl_gazebo-classic', 'worlds', 'empty.world'
    )

    # Use different sdf files with different ports set inside
    uav_model_path_0 = os.path.join(dsls_share_dir, 'models', 'px4vision_ancl', 'px4vision_ancl_0.sdf')
    uav_model_path_1 = os.path.join(dsls_share_dir, 'models', 'px4vision_ancl', 'px4vision_ancl_1.sdf')
    load_model_path = os.path.join(dsls_share_dir, 'models', 'double_slung_load_point_mass', 'double_slung_load_point_mass.sdf')

    env_vars = [
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(HOME, 'PX4-Autopilot', 'Tools', 'simulation', 'gazebo-classic', 'sitl_gazebo-classic', 'models')
            + ':' + os.path.join(dsls_share_dir, 'models')
        ),
    ]

    world_arg = DeclareLaunchArgument('world', default_value=world_path)
    dsl_sdf_arg = DeclareLaunchArgument('dsl_sdf', default_value=load_model_path)

    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true',
            'pause': 'true',
        }.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py'))
    )

    spawn_uav0 = ExecuteProcess(
        cmd=[
            "gz", "model",
            "--spawn-file", uav_model_path_0,
            "--model-name", "px4vision_0",
            "-x", "0.0", "-y", "0.85", "-z", "0.0",
            "-R", "0.0", "-P", "0.0", "-Y", "0.0"
        ],
        output='screen',
        prefix="bash -c 'sleep 5; $0 $@'"
    )

    px4_sitl_uav0 = ExecuteProcess(
        cmd=[
            os.path.join(px4_build_dir, 'bin', 'px4'),
            os.path.join(HOME, 'PX4-Autopilot', 'ROMFS', 'px4fmu_common'),
            '-s', os.path.join(HOME, 'PX4-Autopilot', 'ROMFS', 'px4fmu_common', 'init.d-posix', 'rcS'),
            '-i', '0',
        ],
        # cwd=os.path.join(PX4_RUN_DIR, "sitl_uav0"),
        cwd=PX4_RUN_DIR,
        shell=True,
        prefix="xterm -hold -e",
        output='screen'
    )

    spawn_uav1 = ExecuteProcess(
        cmd=[
            "gz", "model",
            "--spawn-file", uav_model_path_1,
            "--model-name", "px4vision_1",
            "-x", "0.0", "-y", "-0.85", "-z", "0.0",
            "-R", "0.0", "-P", "0.0", "-Y", "0.0"
        ],
        output='screen',
        prefix="bash -c 'sleep 5; $0 $@'"
    )

    px4_sitl_uav1 = ExecuteProcess(
        cmd=[
            os.path.join(px4_build_dir, 'bin', 'px4'),
            os.path.join(HOME, 'PX4-Autopilot', 'ROMFS', 'px4fmu_common'),
            '-s', os.path.join(HOME, 'PX4-Autopilot', 'ROMFS', 'px4fmu_common', 'init.d-posix', 'rcS'),
            '-i', '1',
        ],
        # cwd=os.path.join(PX4_RUN_DIR, "sitl_uav1"),
        cwd=PX4_RUN_DIR,
        shell=True,
        prefix="xterm -hold -e",
        output='screen'
    )

    spawn_load = ExecuteProcess(
        cmd=[
            "gz", "model",
            "--spawn-file", LaunchConfiguration('dsl_sdf'),
            "--model-name", "slung_load"
        ],
        output='screen',
        prefix="bash -c 'sleep 5; $0 $@'"
    )

    xrce_dds_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    return LaunchDescription([
        *env_vars,
        world_arg,
        dsl_sdf_arg,
        gzserver,
        gzclient,
        spawn_uav0,
        px4_sitl_uav0,
        spawn_uav1,
        px4_sitl_uav1,
        spawn_load,
        xrce_dds_agent,
    ])
