"""启动虚拟平台的三个单元（或其中一部分）。

例：
  ros2 launch vsim_ros vsim.launch.py                         # 单摆，三个单元同机
  ros2 launch vsim_ros vsim.launch.py model:=two_link
  ros2 launch vsim_ros vsim.launch.py io:=can                 # I/O 走 vcan0（接口层级 I2）
  ros2 launch vsim_ros vsim.launch.py io:=can can_iface:=can0 # 跨容器 vxcan 链路（scripts/vcan_link.sh）
  ros2 launch vsim_ros vsim.launch.py units:=ecu              # 容器里每个容器只启动一个单元
  ros2 launch vsim_ros vsim.launch.py units:=sim_host,ecu,planner,monitor   # 附带无界面健康检查
  ros2 launch vsim_ros vsim.launch.py params_file:=/path/my_overrides.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    share = get_package_share_directory('vsim_ros')
    model = LaunchConfiguration('model').perform(context)
    io = LaunchConfiguration('io').perform(context)
    units = [u.strip() for u in LaunchConfiguration('units').perform(context).split(',') if u.strip()]
    extra = LaunchConfiguration('params_file').perform(context)
    can_iface = LaunchConfiguration('can_iface').perform(context)

    params = [os.path.join(share, 'config', f'{model}.yaml')]
    if io == 'can':
        params.append(os.path.join(share, 'config', 'can_overlay.yaml'))
    elif io != 'ros':
        raise RuntimeError(f"io must be 'ros' or 'can', got '{io}'")
    if extra:
        params.append(extra)  # 后面的文件覆盖前面的

    known = {'sim_host', 'ecu', 'planner', 'monitor'}
    nodes = []
    for unit in units:
        if unit not in known:
            raise RuntimeError(f"unknown unit '{unit}' (expected {sorted(known)})")
        node_params = list(params)
        if can_iface and unit in ('sim_host', 'ecu'):
            node_params.append({'io.can.interface': can_iface})  # 覆盖 YAML 中的 CAN 接口名
        nodes.append(Node(
            package='vsim_ros',
            executable=f'{unit}_node',
            name=unit,
            output='screen',
            emulate_tty=True,
            parameters=node_params,
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='pendulum', description='pendulum | two_link'),
        DeclareLaunchArgument('io', default_value='ros', description='ros（I1，ROS 话题）| can（I2，vcan0）'),
        DeclareLaunchArgument('units', default_value='sim_host,ecu,planner',
                              description='要启动的单元，逗号分隔'),
        DeclareLaunchArgument('params_file', default_value='', description='额外的参数覆盖文件'),
        DeclareLaunchArgument('can_iface', default_value='',
                              description='io:=can 时的 CAN 接口名（空 = 用 can_overlay.yaml 里的 vcan0）'),
        OpaqueFunction(function=_setup),
    ])
