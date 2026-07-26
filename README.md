# Mid-360 Driver 🚀

<img src="https://img.shields.io/badge/ROS2-Humble-blue" /> <img src="https://img.shields.io/badge/C%2B%2B-20-purple" /> <img src="https://img.shields.io/badge/Asio-1.18-green" />

> 一个轻量级的 Mid-360 LiDAR ROS2 驱动，基于 Asio C++20 协程直连 UDP 🎯
> 不依赖 Livox-SDK，不依赖 Boost，干干净净～ ✨

本项目的设计参考了以下两个项目，并在此基础上做了大量优化与精简：

- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) — Livox 官方驱动，功能全面但依赖 Livox-SDK2，架构较重 📦
- [mid360_driver](https://github.com/EpsilonCygniLi/mid360_driver) — 社区实现，支持多雷达和坐标变换，使用原始 Asio socket 🌟

通信协议细节参考 [Livox Mid-360 官方文档](https://livox-wiki-cn.readthedocs.io/zh-cn/latest/tutorials/new_product/mid360/mid360.html) 📖

## 做了什么优化？🤔

相比上述参考项目，主要做了以下改进：

| 优化点 | 说明 |
|--------|------|
| Asio C++20 协程 🧵 | 用 `co_await` 替代手动 `select`/`epoll` 轮询，异步代码像同步一样写 |
| CRC 编译期查表 ⚡ | 256 项 LUT 在编译期生成，消除逐 bit 循环 |
| Theta 缓存 + Phi LUT 📊 | 球坐标模式下 400K 次/秒 trig 调用 → 0 次，288KB 空间换时间 |
| 双缓冲 + move 语义 🔄 | io-thread ↔ timer-thread 零拷贝交接，发布时 swap 指针 |
| 预计算平方阈值 📐 | 点云过滤时 d² 比较取代 sqrt，省去每点一次函数调用 |
| FIFO 截断防溢出 🪣 | 缓冲打满时丢弃最旧数据而非全部清空，保住最新帧 |
| Debug 日志系统 📝 | JSON lines 格式 `.trc` 文件，带时间戳子目录，方便 agent 自动分析 |
| 无 SDK 依赖 🧹 | 只需 `sudo apt install libasio-dev`，一行命令完成环境准备 |

## 安装依赖

```bash
sudo apt install libasio-dev
```

## 快速使用 🚗

```bash
colcon build --packages-select mid360_driver
source install/setup.bash
ros2 launch mid360_driver mid360_driver.launch.py
```

## 参数配置 ⚙️

编辑 `config/param.yaml`：

```yaml
mid360_driver:
  ros__parameters:
    host_ip: "192.168.32.80"       # 本机有线网 IP
    lidar_ip: "auto"                # "auto"=广播发现 / 具体 IP=直连
    pcl_data_type: 1                # 1=int32, 2=int16, 3=球坐标
    publish_freq_hz: 10             # 点云发布频率 (0=不限)
    imu_data_en: true               # IMU 开关
    detect_mode: 0                  # 0=正常, 1=敏感
    publish_tf: true                # lidar→imu 静态 TF
```

## 输出话题 📡

| 话题 | 类型 | 频率 | 字段 |
|------|------|------|------|
| `/lidar/mid360/lidar` | PointCloud2 | 10Hz | x,y,z,intensity,timestamp,tag |
| `/lidar/mid360/imu` | Imu | 200Hz | 6 轴陀螺+加速度计 |
| `/tf_static` | TFMessage | 启动时 | IMU 外参偏移 |

## 服务 🛎️

| 服务 | 类型 | 说明 |
|------|------|------|
| `~/enable_sampling` | SetBool | true=采样 / false=待机 |
| `~/get_work_state` | Trigger | 查询当前工作状态 |


## ⏰ PTP 时间同步

Mid-360 雷达内置 IEEE 1588v2.0 PTP 从时钟（slave）🎯，当网络中存在 PTP 主时钟时会自动锁定时间同步。
驱动检测到 `time_type=1`（PTP）的数据后会自动切换为 PTP 时间戳路径。

### 🔍 判断网卡是否支持 PTP

```bash
ethtool -T enp170s0
```

| 输出特征 | 结论 |
|---------|------|
| 包含 `hardware-transmit/receive` + `PTP Hardware Clock` | 支持硬件 PTP ✅ |
| 只有 `software-transmit/receive`，无 `PTP Hardware Clock` | 仅支持软件 PTP |

### 🚀 启动 PTP 主时钟

```bash
# 软件时间戳模式（推荐，兼容性好）
sudo ptp4l -i enp170s0 -S

# 硬件时间戳模式（需网卡+驱动支持）
# 本机 igc 驱动有 tx_timestamp bug，硬件模式不可用
# sudo ptp4l -i enp170s0 -H
```

建议配置为 systemd 服务开机自启：

```bash
sudo tee /etc/systemd/system/ptp4l.service << 'EOF'
[Unit]
Description=PTP IEEE 1588 Master Clock
After=network.target

[Service]
ExecStart=/usr/sbin/ptp4l -i enp170s0 -S
Restart=always

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable ptp4l
sudo systemctl start ptp4l
```


> 💡 **驱动检测到 `time_type=1`（PTP）的数据后会自动切换为 PTP 时间戳路径**
