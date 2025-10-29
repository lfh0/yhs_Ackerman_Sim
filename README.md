# YHS Car Simulation 说明文档

## 1. 启动文件
仿真启动文件：`roslaunch fw-mid yhs_gazebo.launch`
键盘控制节点：`rosrun fw-mid keyboard_controll`

## 2. yhs小车选择档位
在 `control.launch` 文件中，找到如下行：
```xml
<param name="gear" type="int" value="6" /> # 4T4D-档
<param name="gear" type="int" value="7" /> # 横移-档
# 在cmdvel2gazebo中将ackerman的格式修改为twist，消息内容如下
# 控制话题：/cmd_vel
geometry_msgs/Twist "linear:
  x: 0.0   # 控制模型前进和后退，前进正，后退负
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0   # 控制模型转向，左转正，右转负
```
## 3. livox mid360雷达选择格式
在 `urdf/fw-mid.xacro` 文件中，修改以下内容：
```xml
<publish_pointcloud_type>2</publish_pointcloud_type> # 4T4D-档
```
`publish_pointcloud_type`：该参数决定了点云发布的格式：

`0`: 点云将以 `sensor_msgs::PointCloud` 消息格式发布。

`1`: 点云将以 `sensor_msgs::PointCloud2` 消息格式发布，字段包括 `x, y, z`。

`2` : 点云将以 `sensor_msgs::PointCloud2` 消息格式发布，字段包括 `x, y, z, intensity, tag, line, timestamp（/sensor_msgs/PointCloud2）`。

`3`(默认值): 点云将以 Livox 自定义消息格式发布`（/livox_laser_simulation/CustomMsg）`。
```xml
offset_time: 
x: 
y: 
z: 
reflectivity: 
tag: 
line: 
```
## 4. livox mid360雷达参数
以10 Hz的频率更新点云，以200 Hz的频率推送IMU数据。IMU数据包含3轴加速度和3轴角速度，其方向与点云坐标相同。IMU芯片在点云坐标中的位置为`x=11.0mm y=23.29mm z=-44.12mm`