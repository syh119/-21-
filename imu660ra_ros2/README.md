# imu660ra_ros2

给 RDK X5 用的 ROS 2 `sensor_msgs/msg/Imu` 驱动包，面向逐飞 `IMU660RA`，实际芯片是 `Bosch BMI270`。

当前版本默认通过 Linux `i2c-dev` 访问传感器，同时保留 `spidev` 作为可选传输方式。默认会上电做一次陀螺仪静态零偏标定，适合直接接入里程计/IMU 融合链路。

## 功能

- 按 BMI270 初始化流程装载 8 KB 配置文件
- 读取三轴加速度和三轴角速度
- 发布 `imu/data_raw`
- 上电自动做陀螺仪零偏标定
- 通过 ROS 2 参数配置 I2C/SPI 设备、地址、量程和发布频率

## I2C 接线

逐飞 IMU660RA 模块：

- `SCL/SPC` -> RDK X5 的 `I2C_SCL`
- `SDA/DSI` -> RDK X5 的 `I2C_SDA`
- `SA0/SDO` -> 地址选择
- `VCC` -> `3.3V`
- `GND` -> `GND`

地址说明：

- `SA0/SDO` 接地：`0x68`
- `SA0/SDO` 上拉：`0x69`

逐飞文档里这个模块默认是 `0x69`。

## 构建

把这个包放进你的 ROS 2 工作区 `src` 下，例如：

```bash
cd ~/ros2_ws/src
ln -s /path/to/IMU660RX_Product-master/imu660ra_ros2 .
cd ..
colcon build --packages-select imu660ra_ros2
source install/setup.bash
```

## 运行

先确认 RDK X5 上已经打开 I2C，并且设备节点存在，例如 `/dev/i2c-1`。

```bash
ros2 launch imu660ra_ros2 imu660ra.launch.py
```

或者直接运行：

```bash
ros2 run imu660ra_ros2 imu660ra_node --ros-args \
  -p transport:=i2c \
  -p device:=/dev/i2c-0 \
  -p i2c_address:=105
```

## 主要参数

- `transport`: 传输方式，默认 `i2c`，可选 `i2c` 或 `spi`
- `device`: 设备节点，I2C 默认 `/dev/i2c-0`
- `i2c_address`: I2C 从地址，默认 `0x69`
- `topic_name`: 发布话题名，默认 `imu/data_raw`
- `spi_speed_hz`: SPI 时钟，仅 `transport=spi` 时使用，默认 `10000000`
- `publish_rate_hz`: ROS 发布频率，默认 `50.0`
- `frame_id`: IMU 消息坐标系，默认 `imu_link`
- `acc_range_g`: 加速度量程，可选 `2/4/8/16`
- `gyro_range_dps`: 角速度量程，可选 `125/250/500/1000/2000`
- `acc_conf`: BMI270 `ACC_CONF` 原始寄存器值，默认 `0xA7`
- `gyr_conf`: BMI270 `GYR_CONF` 原始寄存器值，默认 `0xA9`
- `config_chunk_size`: 装载 BMI270 配置文件时的分块长度，默认 `64`
- `calibrate_gyro_on_startup`: 是否上电自动做陀螺仪零偏标定，默认 `true`
- `gyro_calibration_duration_sec`: 陀螺仪静止采样时长，默认 `2.0`
- `gyro_calibration_settle_sec`: 初始化后等待传感器稳定的时间，默认 `0.5`
- `gyro_offset_x/y/z`: 手动附加的陀螺仪偏置，单位 `rad/s`
- `acc_offset_x/y/z`: 手动附加的加速度偏置，单位 `m/s^2`
- `enable_online_bias_calibration`: 运行时是否在静止状态下继续修正陀螺零偏，默认 `true`
- `online_bias_time_constant_sec`: 在线零偏收敛时间常数，越小收敛越快，默认 `8.0`
- `stationary_accel_norm_threshold_mps2`: 判定静止时允许的加速度模长误差，默认 `0.2`
- `stationary_gyro_threshold_radps`: 判定静止时允许的角速度模长，默认 `0.08`
- `gyro_zero_deadband_radps`: 输出角速度死区，小于该值直接压到 0，默认 `0.01`

## 输出说明

- `orientation` 未提供，`orientation_covariance[0] = -1`
- `linear_acceleration` 单位是 `m/s^2`
- `angular_velocity` 单位是 `rad/s`
- 发布的是去掉陀螺仪零偏后的角速度，便于直接替换现有融合中的 IMU 输入
- 启用在线零偏修正后，车体静止时会缓慢更新零偏，降低长时间运行后的原地零飘

## 可选 SPI

如果你后面改成 SPI，也可以直接用同一个节点：

```bash
ros2 run imu660ra_ros2 imu660ra_node --ros-args \
  -p transport:=spi \
  -p device:=/dev/spidev1.0
```

## 注意

- BMI270 配置文件装载在一次上电后不应重复执行太多次；如果反复重启节点但模块没断电，异常时建议给模块重新上电。
- 如果初始化失败，优先检查 I2C 设备节点、供电、地线、地址和上拉是否正确。
- 自动零偏标定期间车体必须静止，否则会把运动角速度当成零偏扣掉。
