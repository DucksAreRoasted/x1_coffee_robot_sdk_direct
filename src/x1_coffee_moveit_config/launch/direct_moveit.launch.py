from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("x1_coffee_moveit_config")
    move_group = PathJoinSubstitution([share, "launch", "move_group.launch.py"])
    rviz = PathJoinSubstitution([share, "launch", "moveit_rviz.launch.py"])
    # 只开规划和 RViz。网口主站由 robot_startup 单独拉起，避免第二个 EtherCAT 主站。
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(move_group)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(rviz)),
    ])
