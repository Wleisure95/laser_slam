# laser-slam
深蓝学院 第一期激光slam 作业完整答案解析

1.  在ros工作空间的src文件夹下执行 git clone git@github.com:Wleisure95/laser_slam.git
2.  共有5个工程，具体分析看每个工程的pdf课件和README文件。

-   [x] calib_odom : 实现了一个基于csm库的icp函数的激光里程计，并用线性最小二乘对里程计做了矫正。
-   [x] laser_undistortion : 实现了利用里程计线性插值的数据，去除激光雷达在运动过程中产生的畸变。依赖于champion_nav_msgs包，一种自定义的激光雷达数据消息类型。如果编译报错，可以先编译champion_nav_msgs包，再编译本身。
-   [x] gmapping : 对gmapping的流程进行了分析。
-   [x] ls_slam :  实现了一个二维位姿的图优化，手写了一个BA。
-   [x] occupany_mapping : 实现了基于已知定位的覆盖栅格建图算法。

