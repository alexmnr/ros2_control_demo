from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression 
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context):
    # Load parameters
    log_level = context.launch_configurations['log_level']
    ns = context.launch_configurations['ns']
    tf_prefix = context.launch_configurations['tf_prefix']
    can_interface = context.launch_configurations['can_interface']
    use_mock_hardware = context.launch_configurations['use_mock_hardware']

    # print parameters
    print("")
    print("Starting driver with paramaters:")
    print(" log_level:           " + log_level)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + "/" + ns)
    print(" can_interface:       " + can_interface)
    print(" use_mock_hardware:   " + use_mock_hardware)
    print("")

    # ros2_control xacro description
    urdf_file_path = PathJoinSubstitution(
            [FindPackageShare("ros2_control_demo"), "config", "robot.xacro"]
            )
    urdf_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                urdf_file_path,
                " ",
                "tf_prefix:=",
                tf_prefix,
                " ",
                "can_interface:=",
                can_interface,
                " ",
                "use_mock_hardware:=",
                use_mock_hardware,
                ]
            )
    robot_description = {
            "robot_description": ParameterValue(urdf_content, value_type=str)
            }

    # ros2_control config
    ros2_controllers_file = PathJoinSubstitution(
            [FindPackageShare("ros2_control_demo"), "config", "ros2_controllers.yaml"]
            )

    # nodes
    nodes = []

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            robot_description,
            {'publish_frequency': 500.0}
            ]
        ))

    nodes.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[
            ParameterFile(ros2_controllers_file, allow_substs=True),
            ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        ))

    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace=ns,
            arguments=[
                "--controller-manager-timeout",
                "10",
            ]
            + inactive_flags
            + controllers
            + ["--ros-args", "--log-level", log_level],
        )

    controllers_active = [
        "joint_state_broadcaster",
    ]
    controllers_inactive = [
        "torque_passthrough_controller",
        "velocity_ramped_controller",
        "velocity_passthrough_controller",
        "position_trajectory_controller",
        "position_filtered_controller",
        "position_passthrough_controller",
    ]

    nodes.append(controller_spawner(controllers_active, True))
    nodes.append(controller_spawner(controllers_inactive, False))

    return nodes

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
            DeclareLaunchArgument(
                'log_level',
                default_value='error',
                description="Log Level to use for all nodes",
                choices=["info", "debug", "error"],
                )
            )
    declared_arguments.append(
            DeclareLaunchArgument(
                'ns',
                default_value='',
                description='namespace of the robot (used as prefix, so needed if running multiple robots)'
                )
            )
    declared_arguments.append(
            SetLaunchConfiguration('tf_prefix', PythonExpression(["'", LaunchConfiguration('ns'), "' + '_' if '", LaunchConfiguration('ns'), "' else ''"]))
            )
    declared_arguments.append(
            DeclareLaunchArgument(
                "can_interface", 
                default_value="can0",
                description="CAN interface name that the hardware interface should use."
                )
            )
    declared_arguments.append(
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Start robot with mock hardware mirroring command to its states.",
                )
            )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

