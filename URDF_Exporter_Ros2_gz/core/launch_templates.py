display_launch = """import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _package_available(package_name: str) -> bool:
    try:
        from ament_index_python.packages import get_package_share_directory

        get_package_share_directory(package_name)
        return True
    except Exception:
        return False


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare(package="urdf_description").find("urdf_description")
    default_model_path = os.path.join(pkg_share, "urdf", "urdf.xacro")
    default_rviz_config_path = os.path.join(pkg_share, "config", "display.rviz")
    default_world_path = os.path.join(pkg_share, "worlds", "room.sdf")
    default_gz_args = f"-r -v 4 {default_world_path}"

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_gz = LaunchConfiguration("start_gz")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    world_name = LaunchConfiguration("world_name")
    entity_name = LaunchConfiguration("entity_name")
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
    )

    joint_state_publisher_node = None
    if _package_available("joint_state_publisher"):
        joint_state_publisher_node = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_joint_state_publisher),
        )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare(package="ros_gz_sim").find("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
        condition=IfCondition(start_gz),
    )

    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"],
        output="screen",
        condition=IfCondition(start_gz),
    )

    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        condition=IfCondition(start_gz),
        arguments=[
            "-world",
            world_name,
            "-name",
            entity_name,
            "-allow_renaming",
            "true",
            "-param",
            "robot_description",
        ],
        parameters=[{"robot_description": robot_description}],
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            name="use_joint_state_publisher",
            default_value="True",
            description="Run joint_state_publisher (disable if /joint_states comes from sim)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="world_name",
            default_value="default",
            description="Gazebo world name (must match the loaded SDF world)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="entity_name",
            default_value="dual_scorpion",
            description="Spawned entity name in Gazebo",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="True",
            description="Flag to enable use_sim_time",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="start_gz",
            default_value=use_sim_time,
            description="Launch Gazebo Sim + spawn the robot (defaults to use_sim_time value)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to robot xacro file",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="gz_args",
            default_value=default_gz_args,
            description="Arguments to be passed to `gz sim`",
        )
    )

    ld.add_action(gz_sim_launch)
    ld.add_action(clock_bridge_node)
    if joint_state_publisher_node is not None:
        ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(rviz_node)
    return ld
"""


"""from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('%s')

    xacro_file = os.path.join(share_dir, 'urdf', '%s.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
"""

gazebo_launch = """from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('%s')

    xacro_file = os.path.join(share_dir, 'urdf', '%s.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
"""



"""from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('%s')

    xacro_file = os.path.join(share_dir, 'urdf', '%s.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')

    default_world = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'worlds',
        'empty.sdf'
    ])

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World SDF file to load'
    )

    declare_gz_args = DeclareLaunchArgument(
        'gz_args',
        default_value=[TextSubstitution(text='-r '), world],
        description='Arguments passed to gz sim (for example: -r <world>.sdf)'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf,
             'use_sim_time': use_sim_time}
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': gz_args
        }.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', '%s',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_gz_args,
        robot_state_publisher_node,
        gz_sim,
        spawn_entity,
    ])
"""


def get_display_launch_text(package_name, robot_name):
    return display_launch % (package_name, robot_name)


def get_gazebo_launch_text(package_name, robot_name):
    return gazebo_launch % (package_name, robot_name, robot_name)
