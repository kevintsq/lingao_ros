# Lingao_bringup 灵遨科技ROS底层通讯驱动

 **NOTE:**

* 驱动使用私有通讯协议
* 带线速度和角速度校准调节比例参数
* 带有基于里程计odom坐标发布功能
* 需要安装ROS环境
* 需要安装基础包

# 使用方式
## 1.安装关联软件包

1. 安装关联ROS包

    tf2 tf2-ros roslint robot-localization

2. 导入lingao_bringup, lingao_msgs， imu_calib包到工作空间
    首先进入工作目录

    ```
    git clone https://e.coding.net/keaa/lingaoros/lingao_msgs.git
    git clone https://e.coding.net/keaa/lingaoros/imu_calib.git
    git clone https://e.coding.net/keaa/lingaoros/lingao_bringup.git
    ```

## 2.导入设备识别符
串口驱动可使用我们的udev识别文件或者更改工作包的驱动号从而启动串口通讯

1. 更改工作包空间串口号,文件所在位置`/lingao_bringup/launch/lingao_base_driver.launch`
    将port后面串口号更改为你现在使用的串口号

## 3.启动文件

1. 单独启动驱动包，同时发布imu信息
    ``` linux
    roslaunch lingao_bringup lingao_base_driver.launch
    ```
2. 单独启动驱动包，不使用imu
    ``` linux
    roslaunch lingao_bringup lingao_base_driver_notuse_imu.launch
    ```
3. 单独启动驱动包，发布imu信息, 发布基于编码器的odom
    ``` linux
    roslaunch lingao_bringup lingao_base_driver_pub_odom.launch
    ```
4. 启动bringup，IMU信息滤波并且和里程计融合，发布TF坐标
    ``` linux
    roslaunch lingao_bringup bringup.launch
    ```
5. 启动robot节点，基于bringup基础上启动雷达和载入机器人模型
    ``` linux
    roslaunch lingao_bringup robot.launch
    ```

# Lingao Bringup API
## 发布的话题
/raw_imu (lingap_msgs/Imu)  
- 主题包括基于加速度和陀螺仪传感器的机器人姿态  

/raw_odom (nav_msgs/Odometry)  
- 基于编码器的的里程表信息

/battery_state (lingao_msgs/Battery)
- 电池电压和电量状态

## 订阅的话题
/cmd_vel (geometry_msgs/Twist)
- 控制机器人平移线速度(单位:m/s)和旋转角速度(单位:rad/s)

## 参数设置 - Parameters
- 驱动带有线速度和角速度校准调节比例参数
    设置文件所在位置`/lingao_bringup/launch/lingao_base_driver.launch`

- ~baud (int, default: 115200)  
设置串口波特率
- ~port (string, default: /dev/lingao)  
设置串口号
- publish_odom (bool, default: false)  
是否发布基于里程计odom
- linear_scale (double, default: 1.0)  
里程计线速度校准比例
- angular_scale (double, default: 1.0)  
里程计角速度校准比例