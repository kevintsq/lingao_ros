# imu_calib

该存储库用于计算校准参数并将其应用于 IMU 测量的工具。  
原始储存库位置：https://github.com/dpkoch/imu_calib

## Usage

该ROS包含两个节点  
第一个节点是计算加速度计校准参数并保存到 YAML 文件中，校准只需要运行一次。
第二个节点读取YAM校准文件并应用到未校准的 IMU 主题以生成校准的 IMU 主题。

## Nodes

### do_calib
加速度计校准节点，应该使用rosrun运行。节点收到第一条 IMU 消息后，会提示您将 IMU 保持在某个方向，然后按 Enter 记录测量值。在所有 6 个方向都完成后，节点将计算校准参数并将它们写入指定的 YAML 文件。

底层算法是基于 [STMicroeletronics 应用笔记AN4508](http://www.st.com/content/ccc/resource/technical/document/application_note/a0/f0/a0/62/3b/69/47/66/DM00119044.pdf/files/DM00119044.pdf/jcr:content/translations/en.DM00119044.pdf).中描述的并类似于该方法的最小二乘校准方法。由于算法的性质，获得良好的校准需要 IMU 沿其每个轴相当准确地定位。

#### Topics

##### 订阅的话题
- `imu/data_raw` (sensor_msgs/onboard_imu) <br>
  未经校准的原始 IMU 测量值

#### Parameters
- `~calib_file` (string, default: "imu_calib.yaml") <br>
  将写入校准参数的文件
- `~measurements` (int, default: 500) <br>
  为每个方向收集的测量数据量
- `~reference_acceleration` (double, default: 9.80665) <br>
  预期的重力加速度

### apply_calib

do_calib 节点计算的加速度计校准参数。还可以选择（默认启用）在启动时计算陀螺仪偏差并将其减去。

#### Topics

##### 订阅 Topics
- `imu/data_raw` (sensor_msgs/onboard_imu) <br>
  未经校准的原始 IMU 测量值

##### 发布 Topics
- `imu/raw_cailb` (sensor_msgs/data_raw) <br>
  发布已校准的 IMU 测量值

#### 参数
- `~calib_file` (string, default: "imu_calib.yaml") <br>
  从中读取校准参数的文件
- `~calibrate_gyros` (bool, default: true) <br>
  是否在启动时计算陀螺偏差并随后减去它们
- `~calibrate_accel` (bool, default: true) <br>
  是否校准加速度传感器
- `~gyro_calib_samples` (int, default: 100) <br>
  用于计算陀螺仪偏差的测量次数
