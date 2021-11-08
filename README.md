#  灵遨科技 - 移动机器人 ROS 软件包

该存储库是灵遨ROS机器人最小软件包集

## ROS Packages
* lingao_base: 灵遨底盘驱动软件包，用于ROS的底盘通讯收发
* lingao_bringup: 灵遨ROS节点启动文件和配置包
* lingao_description: 灵遨机器人URDF模型    
* lingao_msgs: 灵遨自定义消息
* imu_calib: 板载imu校准包

## 灵遨科技ROS包的安装
1. 安装ROS依赖包
    ``` linux
    sudo apt-get install -y \
    ros-$ROS_DISTRO-roslint \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-imu-filter-madgwick \
    ros-$ROS_DISTRO-teleop-twist-keyboard
    ```

2. 导入lingao_ros包到工作空间
    ``` linux
    $ mkdir -p ~/lingao_ws/src/
    $ cd ~/lingao_ws/src/
    $ catkin_init_workspace
    $ git clone https://e.coding.net/keaa/lingaoros/lingao_ros.git
    $ catkin_make
    ```

## 通讯接口设置
串口驱动可使用我们的udev识别文件或者更改工作包的驱动号从而启动串口通讯

1. 方法一：更改串口号，并且给予串口权限
    更改工作包空间串口名,文件所在位置`/lingao_ros/lingao_base/launch/lingao_base_driver.launch`
    将port_name后面串口号`/dev/lingao`更改为你现在使用的串口号，并且`给予串口权限`

2. 方法二（推荐）：导入lingao设备识别符到系统
    ``` linux
    $ source devel/setup.bash
    $ roscd lingao_base
    $ sudo cp 50-lingao.rules /etc/udev/rules.d/
    ``` 

## ROS Package 基本用法
下面是lingao_bringup的启动包，有下面几种（未全列出

1. 单独启动底盘驱动包
    ``` linux
    roslaunch lingao_base lingao_base_driver.launch
    ```
    所述 lingao_base/launch/lingao_base_driver.launch 有4个参数：
    * imu_use: 是否发布imu信息(default: false)
    * pub_odom_tf: 是否发布里程计TF转换(default: false)
    * linear_scale: 线速度计校准比例(default: 1.0)
    * angular_scale: 角速度计校准比例(default: 1.0)
    更多配置可参考lingao_base里的README.md

2. 启动bringup，IMU信息滤波并且和里程计融合，发布TF坐标
    ``` linux
    roslaunch lingao_bringup bringup.launch
    ```
3. 启动robot节点，基于bringup基础上启动雷达和载入机器人模型
    ``` linux
    roslaunch lingao_bringup robot.launch
    ```

* 注意： 需在连接通讯端工控启动，选择以上任意一个运行，退出后可运行另外的包

## 使用测试
首先运行驱动包

1. 发布 Twist 消息控制小车运动
    ``` linux
    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.0}}'
    ```

2. 使用键盘控制机器人
    ``` linux
    roslaunch lingao_bringup lingao_teleop_keyboard.launch
    ```
    现在可通过下面键盘按键移动机器人：  
    u       i     o  
    j       k     l  
    m       ,        .  

    改变速度：
    q/z : 增加/减少最大速度 10%  
    w/x : 仅增加/减少 10% 的线速度  
    e/c : 仅增加/减少 10% 的角速度  


* 安全注意事项：
键盘控制节点默认速度值很高，请确保在开始用键盘控制机器人之前降低速度命令！随时准备好您的遥控器来接管控制。
发布错误的Twist速度消息可能使机器人以最快速度运行，你可以随时准备好您的遥控器来接管控制。


