teb_local_planner ROS 软件包
=============================

teb_local_planner 软件包实现了 2D 导航堆栈中 base_local_planner 的一个插件。
其底层方法称为时间弹性带(Timed Elastic Band)，可在运行时局部优化机器人的轨迹，考虑轨迹执行时间、
与障碍物的距离以及运动学和动力学约束的遵守情况。

更多信息和教程请参考 http://wiki.ros.org/teb_local_planner

*melodic-devel* 分支的构建状态：
- ROS Buildfarm (Melodic): [![Melodic Status](http://build.ros.org/buildStatus/icon?job=Mdev__teb_local_planner__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__teb_local_planner__ubuntu_bionic_amd64/)

### 移植到 ROS2
此分支是移植到 ROS2(Dashing Diademata) 的 teb_local_planner 软件包。目前它与 [Navigation2(master 分支)](https://github.com/ros-planning/navigation2/tree/master)([226f06c](https://github.com/ros-planning/navigation2/commit/226f06ce282c727ca240ce8be0cb4b093e26343b)) 兼容。您可以通过启动以下命令来测试 teb_local_planner 与 Navigation2 和 TurtleBot3 仿真：
```
ros2 launch teb_local_planner teb_tb3_simulation_launch.py
```

## 引用本软件

*由于开发投入了大量时间和精力，如果您在自己的研究中使用本规划器，请至少引用以下出版物之一：*

- C. Rösmann, F. Hoffmann and T. Bertram: Integrated online trajectory planning and optimization in distinctive topologies, Robotics and Autonomous Systems, Vol. 88, 2017, pp. 142–153.
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Trajectory modification considering dynamic constraints of autonomous robots. Proc. 7th German Conference on Robotics, Germany, Munich, May 2012, pp 74–79.
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Efficient trajectory optimization using a sparse model. Proc. IEEE European Conference on Mobile Robots, Spain, Barcelona, Sept. 2013, pp. 138–143.
- C. Rösmann, F. Hoffmann and T. Bertram: Planning of Multiple Robot Trajectories in Distinctive Topologies, Proc. IEEE European Conference on Mobile Robots, UK, Lincoln, Sept. 2015.
- C. Rösmann, F. Hoffmann and T. Bertram: Kinodynamic Trajectory Optimization and Control for Car-Like Robots, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Vancouver, BC, Canada, Sept. 2017.

<a href="https://www.buymeacoffee.com/croesmann" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/lato-black.png" alt="Buy Me A Coffee" height="31px" width="132px" ></a>

## 视频

以下视频中的左侧视频展示了该软件包的功能，并显示了仿真和真实机器人场景中的示例。
视频的音轨中包含一些口头解释。
右侧视频演示了 0.2 版本引入的功能（支持类车机器人和代价地图转换）。请先观看左侧视频。

<a href="http://www.youtube.com/watch?feature=player_embedded&v=e1Bw6JOgHME" target="_blank"><img src="http://img.youtube.com/vi/e1Bw6JOgHME/0.jpg" 
alt="teb_local_planner - An Optimal Trajectory Planner for Mobile Robots" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=o5wnRCzdUMo" target="_blank"><img src="http://img.youtube.com/vi/o5wnRCzdUMo/0.jpg" 
alt="teb_local_planner - Car-like Robots and Costmap Conversion" width="240" height="180" border="10" /></a>

## 许可证

*teb_local_planner* 软件包采用 BSD 许可证。
它依赖于其他 ROS 软件包，这些软件包在 package.xml 中列出。它们也采用 BSD 许可证。

包含的一些第三方依赖项采用不同的许可条款：
 - *Eigen*，MPL2 许可证，http://eigen.tuxfamily.org
 - *libg2o* / *g2o* 本身采用 BSD 许可证，但启用的 *csparse_extension* 采用 LGPL3+ 许可证，
   https://github.com/RainerKuemmerle/g2o。[*CSparse*](http://www.cise.ufl.edu/research/sparse/CSparse/) 作为 *SuiteSparse* 集合的一部分包含在内，http://www.suitesparse.com。
 - *Boost*，Boost 软件许可证，http://www.boost.org

包含的所有软件包均以"按原样"分发，期望它们能够有用，但不提供任何明示或暗示的保证；不包括对适销性或特定用途适用性的暗示保证。有关更多详细信息，请参阅许可证。

## 依赖项

使用 *rosdep* 安装依赖项（在 *package.xml* 和 *CMakeLists.txt* 文件中列出）：

    rosdep install teb_local_planner


