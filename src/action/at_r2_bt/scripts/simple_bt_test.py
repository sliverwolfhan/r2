#!/usr/bin/env python3
"""
最简单的行为树测试脚本
直接加载并运行行为树，不需要 Nav2 的其他组件
"""

import rclpy
from rclpy.node import Node
import py_trees
from behaviortree_cpp_v4 import BehaviorTreeFactory, NodeStatus
import os
from ament_index_python.packages import get_package_share_directory

def main():
    rclpy.init()
    node = Node('simple_bt_test')
    
    # 获取行为树文件路径
    pkg_dir = get_package_share_directory('nav2_bt_publish_goal')
    bt_xml = os.path.join(pkg_dir, 'behavior_trees', 'example_single_goal.xml')
    
    node.get_logger().info(f'加载行为树: {bt_xml}')
    
    # 创建行为树工厂并注册插件
    factory = BehaviorTreeFactory()
    
    # 注册我们的自定义节点
    factory.registerFromPlugin(get_package_share_directory('nav2_bt_publish_goal') + '/lib/libnav2_bt_publish_goal.so')
    
    # 从 XML 创建树
    tree = factory.createTreeFromFile(bt_xml)
    
    node.get_logger().info('开始执行行为树...')
    
    # 执行行为树
    status = tree.tickWhileRunning()
    
    if status == NodeStatus.SUCCESS:
        node.get_logger().info('✓ 行为树执行成功!')
    else:
        node.get_logger().error(f'✗ 行为树执行失败: {status}')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
