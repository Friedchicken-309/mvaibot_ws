# 注意当前代码依赖ROS2版本为humble，其他版本请尝试构建验证
--- 
# mvaibot底盘控制工作空间

## mvaibot_ws

移动机器人 ROS2 工作空间，基于手柄遥控实现底盘 485 串口通信控制。

## 功能

- 订阅 `/joy` 手柄话题，解析摇杆和按键输入
- 支持多种控制模式切换（遥控 / 全向移动 / 原地转向 / 双阿克曼）
- 速度和转向角独立控制，支持加速档
- 按 50Hz 频率通过 RS485 串口下发 8 字节控制帧
- 使能开关保护

## 硬件

- 控制器：北通阿修罗 3S 手柄
>**使用其他手柄请在自行检查手柄按键映射并修改YAML文件，启动节点时带参数启动，或修改节点内的参数声明默认值**
- 通信：USB 转 RS485 收发器（`/dev/ttyACM0`，波特率 9600）
>**使用其他连接器请在系统终端中查阅设备树，确认端口是/dev/ttyUSB*或其他，在节点中修改**

## 依赖

- ROS2 (Humble / Iron / Jazzy)
- `ros2` joy 包：`sudo apt install ros-$ROS_DISTRO-joy`

## 构建

```bash
git clone https://github.com/Friedchicken-309/mvaibot_ws.git
cd mvaibot_ws
colcon build --packages-select mvaibot_485_driver
source install/setup.bash
```

## 运行

```bash
# 终端1：启动手柄节点
ros2 run joy joy_node

# 终端2：启动驱动节点
ros2 run mvaibot_485_driver mvaibot_485_driver

# 或带参数文件启动
ros2 run mvaibot_485_driver mvaibot_485_driver \
  --ros-args --params-file src/mvaibot_485_driver/src/mvaibotjoy.yaml
```

## 手柄按键映射

| 按键 | 功能 |
|------|------|
| 左摇杆上下 | 速度控制（向前推加速） |
| 左摇杆左右 | 转向角度（0° - 180°，中位 90°） |
| A | 切换行驶方向（前进 / 后退） |
| B | 切换限位状态 |
| X | 切换控制模式 |
| LT（按住） | 运动使能（松开速度归零） |
| RT（按住） | 加速档 |

## 串口协议

8 字节帧格式：

| 字节 | 内容 | 说明 |
|------|------|------|
| [0] | `0x33` | 起始位 |
| [1] | mode | 控制模式：0 遥控，1 全向，2 原地转向，3 双阿克曼 |
| [2] | dir | 行驶方向：0 停止，1 后退，2 前进 |
| [3] | speed | 速度：0 - 100 |
| [4] | angle | 方向角：0 - 180 |
| [5] | limit | 限位状态：0 正常，1 前限位，2 后限位，3 急停 |
| [6] | `0x00` | 预留 |
| [7] | sum | 前 7 字节之和的低八位 |

## 目录结构

```
mvaibot_ws/
├── src/
│   └── mvaibot_485_driver/
│       ├── src/
│       │   ├── mvaibot_485_driver.cpp    # 驱动节点源码
│       │   └── mvaibotjoy.yaml           # 参数配置文件
│       ├── CMakeLists.txt
│       └── package.xml
├── build/
├── install/
└── log/
```

## License

Apache-2.0

--- 

## TODO LIST

- [ ] 把速度控制的消息接口由/joy迁移到/cmd_vel，状态与模式控制接口保留在/joy，小车速度控制帧中100对应的物理速度是8m/s
- [ ] 连接真机测试，研究全向模式和双阿克曼模式的具体运动模型，特别是前进方向代表车轮转向还是车身航向
- [ ] 搭建机器人三维模型和URDF文件
- [ ] 后续开发

