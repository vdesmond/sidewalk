"""N robots + the ns-3 bridge. Start ns-3 separately (or via scripts/cosim.sh):
   sidewalk-robots --numRobots=<N> --cosimPort=<port> ...
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn(context):
    n = int(LaunchConfiguration("num_robots").perform(context))
    period = int(LaunchConfiguration("period_ms").perform(context))
    area = float(LaunchConfiguration("area_size").perform(context))
    nodes = [
        Node(package="sidewalk_cosim", executable="robot_node", name=f"robot_{i}",
             parameters=[{"id": i, "period_ms": period, "area_size": area,
                          "speed": float(LaunchConfiguration("speed").perform(context))}])
        for i in range(n)
    ]
    bridge = Node(package="sidewalk_cosim", executable="bridge_node", name="sidewalk_bridge",
                      output="screen",
                      parameters=[{"num_robots": n,
                                   "port": int(LaunchConfiguration("port").perform(context)),
                                   "step_ms": int(LaunchConfiguration("step_ms").perform(context)),
                                   "duration_s": float(LaunchConfiguration("duration_s").perform(context)),
                                   "realtime": LaunchConfiguration("realtime").perform(context) == "true",
                                   "warmup_s": float(LaunchConfiguration("warmup_s").perform(context)),
                                   "csv_path": LaunchConfiguration("csv_path").perform(context)}])
    nodes.append(bridge)
    # the bridge ends the run (duration_s reached or ns-3 gone): take the robots down with it
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=[Shutdown()])))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="10"),
        DeclareLaunchArgument("period_ms", default_value="100"),
        DeclareLaunchArgument("area_size", default_value="50.0"),
        DeclareLaunchArgument("speed", default_value="1.5"),
        DeclareLaunchArgument("port", default_value="8000"),
        DeclareLaunchArgument("step_ms", default_value="10"),
        DeclareLaunchArgument("duration_s", default_value="0.0"),
        DeclareLaunchArgument("realtime", default_value="true"),
        DeclareLaunchArgument("warmup_s", default_value="2.1"),
        DeclareLaunchArgument("csv_path", default_value="/tmp/sidewalk-cosim.csv"),
        OpaqueFunction(function=spawn),
    ])
