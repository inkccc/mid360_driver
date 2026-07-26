#pragma once
// ============================================================================
//  Mid-360 驱动 — 协议常量 · 数据结构 · CRC · 核心驱动 & ROS2 节点声明
//  (Google C++ Style, 4空格缩进)
// ============================================================================

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define ASIO_NO_DEPRECATED
#include <asio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mid360_driver {

// ────────────────────────────────────────────────────────────────────────────
//  协议常量 (k前缀, PascalCase)
// ────────────────────────────────────────────────────────────────────────────
//  Mid-360 雷达在固定端口监听控制/数据指令。
//  主机端通过端口扫描递增偏移量来避免与同网段其他实例冲突。

// 雷达端固定监听端口
constexpr uint16_t kPortDiscoveryRadar  = 56000;   // 设备发现广播端口
constexpr uint16_t kPortControlRadar    = 56100;   // 控制指令端口
constexpr uint16_t kPortPointCloudRadar = 56300;   // 点云数据推送端口
constexpr uint16_t kPortImuRadar        = 56400;   // IMU 数据推送端口

// 主机端默认端口 (从 56301/56401 起, 被占用则递增扫描)
constexpr uint16_t kHostPointCloudBase = 56301;   // 主机端点云接收入口
constexpr uint16_t kHostImuBase        = 56401;   // 主机端 IMU 接收入口
constexpr uint16_t kHostPushBase       = 56201;   // 主机端状态推送接收入口 (保留, 未启用)
constexpr uint16_t kHostControl        = 56101;   // 主机端控制端口 (保留, sendCommand 使用临时端口)

// 全局阈值
constexpr uint16_t kPortScanLimit      = 50;      // 端口扫描最大偏移量
constexpr uint16_t kCmdSof             = 0xAA;    // 指令起始标志
constexpr uint16_t kCmdVersion         = 0;       // 协议版本
constexpr uint16_t kCmdHeaderSize      = 24;      // CommandHeader 固定长度
constexpr uint16_t kCmdDataMaxLen      = 1400;    // 指令数据段最大长度 (MTU 约束)
constexpr size_t   kUdpBufSize         = 1500;    // UDP 接收缓冲区大小 (≥ 以太网 MTU)
constexpr int      kBindRetryPeriodMs  = 500;     // 端口绑定重试间隔 (2 Hz)
constexpr uint32_t kPointsPerSecond    = 200000;  // 雷达标称点云速率
constexpr uint32_t kImuRateHz          = 200;     // IMU 推送频率
constexpr size_t   kPointBufferReserve = 1024;    // 解析缓冲初始容量 (> 典型 dotNum)
constexpr size_t   kMaxBufferPoints    = 250000;  // pclBufferA_ 绝对上限 (防止无限增长)
constexpr size_t   kThetaCacheCapacity = 64;      // 球坐标 theta 缓存条目数
constexpr size_t   kPhiLutSize         = 36001;   // 球坐标 phi LUT (0~360.00°, 0.01°步)

// 数据类型枚举 (DataHeader.dataType 取值)
enum DataType : uint8_t {
    DATA_TYPE_IMU            = 0x00,   // IMU 数据包
    DATA_TYPE_CARTESIAN_HIGH = 0x01,   // 直角坐标高精度 (int32, 1mm/bit)
    DATA_TYPE_CARTESIAN_LOW  = 0x02,   // 直角坐标低精度 (int16, 10mm/bit)
    DATA_TYPE_SPHERICAL      = 0x03,   // 球坐标 (深度 + theta + phi)
};

// 时间戳同步类型 (DataHeader.timeType 取值)
enum TimestampType : uint8_t {
    TIMESTAMP_NO_SYNC = 0,   // 未同步 — 驱动自动锚定到本机时钟
    TIMESTAMP_PTP     = 1,   // IEEE 1588 PTP 同步
    TIMESTAMP_GPS     = 2,   // GPS 授时同步
};

// 控制指令 ID
enum CmdId : uint16_t {
    CMD_DISCOVERY     = 0x0000,   // 设备发现
    CMD_PARAM_CONFIG  = 0x0100,   // 参数写入
    CMD_PARAM_QUERY   = 0x0101,   // 参数查询
    CMD_REBOOT        = 0x0200,   // 重启
    CMD_GPS_TIME_SET  = 0x0202,   // GPS 时间设置
};

// 参数 Key (用于 writeParams / queryParams)
enum ParamKey : uint16_t {
    KEY_PCL_DATA_TYPE  = 0x0000,   // 点云数据类型
    KEY_LIDAR_IPCFG    = 0x0004,   // 雷达自身 IP 配置
    KEY_STATE_IPCFG    = 0x0005,   // 状态数据推送目标
    KEY_PCL_IPCFG      = 0x0006,   // 点云数据推送目标
    KEY_IMU_IPCFG      = 0x0007,   // IMU 数据推送目标
    KEY_DETECT_MODE    = 0x0018,   // 探测模式
    KEY_WORK_TGT_MODE  = 0x001A,   // 目标工作模式
    KEY_IMU_DATA_EN    = 0x001C,   // IMU 数据使能
    KEY_SN             = 0x8000,   // 序列号 (只读)
    KEY_CUR_WORK_STATE = 0x8006,   // 当前工作状态 (只读)
};

// 雷达工作状态
enum WorkState : uint8_t {
    STATE_SAMPLING    = 0x01,   // 采样中
    STATE_STANDBY     = 0x02,   // 待机
    STATE_ERROR       = 0x04,   // 错误
    STATE_SELF_CHECK  = 0x05,   // 自检中
    STATE_MOTOR_START = 0x06,   // 电机启动中
    STATE_UPGRADE     = 0x08,   // 固件升级中
    STATE_READY       = 0x09,   // 就绪 (可接收控制指令)
};

// 雷达可请求的工作模式 (用于 setWorkMode)
enum WorkMode : uint8_t {
    WORK_MODE_SAMPLING = 0x01,   // 采样模式
    WORK_MODE_STANDBY  = 0x02,   // 待机模式
    WORK_MODE_READY    = 0x09,   // 就绪模式
};

// 控制指令返回码
enum RetCode : uint8_t {
    RET_SUCCESS           = 0x00,   // 成功
    RET_FAILURE           = 0x01,   // 通用失败
    RET_NOT_PERMIT_NOW    = 0x02,   // 当前状态不允许
    RET_OUT_OF_RANGE      = 0x03,   // 参数超出范围
    RET_PARAM_NOT_SUPPORT = 0x20,   // 不支持该参数
    RET_REBOOT_OK         = 0x21,   // 配置已接受, 需重启生效
    RET_RD_ONLY           = 0x22,   // 只读参数不可写
    RET_BAD_LEN           = 0x23,   // 数据长度不匹配
    RET_KEY_MISMATCH      = 0x24,   // Key 校验不匹配
};

// 工作状态 → 可读名称
inline const char* workStateName(uint8_t state) {
    switch (state) {
        case STATE_SAMPLING:    return "SAMPLING";
        case STATE_STANDBY:     return "STANDBY";
        case STATE_ERROR:       return "ERROR";
        case STATE_SELF_CHECK:  return "SELF_CHECK";
        case STATE_MOTOR_START: return "MOTOR_START";
        case STATE_UPGRADE:     return "UPGRADE";
        case STATE_READY:       return "READY";
        default:                return "UNKNOWN";
    }
}

// IMU 芯片在雷达坐标系中的外参偏移 (手册给定, 单位 m)
constexpr float kImuOffsetX = 0.011f;
constexpr float kImuOffsetY = 0.02329f;
constexpr float kImuOffsetZ = -0.04412f;

// ────────────────────────────────────────────────────────────────────────────
//  协议结构体 (紧凑布局, 1字节对齐)
// ────────────────────────────────────────────────────────────────────────────
//  Mid-360 Radar ↔ Host 的控制/数据帧均为二进制协议, 固定字段长度, 无填充。

#pragma pack(push, 1)

// 数据包头 — 36 字节, 位于每个 UDP 数据包的开头
struct DataHeader {
    uint8_t       version;           // 协议版本
    uint16_t      length;            // UDP 负载总长度
    uint16_t      timeInterval;      // 时间间隔 (0.1 μs 单位, 见 parsePointCloud)
    uint16_t      dotNum;            // 本包包含的点数
    uint16_t      udpCnt;            // 本帧 UDP 序列号
    uint8_t       frameCnt;          // 帧计数
    DataType      dataType;          // 数据类型
    TimestampType timeType;          // 时间戳同步类型
    uint8_t       reserved[12];      // 保留字段
    uint32_t      crc32;             // CRC-32 校验 (覆盖 data 段)
    uint64_t      timestamp;         // 绝对时间戳 (ns)
};
static_assert(sizeof(DataHeader) == 36);

// 直角坐标高精度点单元 — 14 字节, int32, 1mm/bit
struct CartesianHighPoint {
    int32_t  x, y, z;                // mm
    uint8_t  reflectivity;           // 反射率 (0~255)
    uint8_t  tag;                    // tag 位域: bit0-5 置信度, bit7 噪声标志
};
static_assert(sizeof(CartesianHighPoint) == 14);

// 直角坐标低精度点单元 — 8 字节, int16, 10mm/bit
struct CartesianLowPoint {
    int16_t  x, y, z;                // 10 mm 单位
    uint8_t  reflectivity;
    uint8_t  tag;
};
static_assert(sizeof(CartesianLowPoint) == 8);

// 球坐标点单元 — 10 字节, 深度 mm + 角度 0.01°
struct SphericalPoint {
    uint32_t depth;                  // 深度距离 (mm)
    uint16_t theta;                  // 仰角 (0.01°, 0=+Z 轴)
    uint16_t phi;                    // 方位角 (0.01°, XY 平面内)
    uint8_t  reflectivity;
    uint8_t  tag;
};
static_assert(sizeof(SphericalPoint) == 10);

// IMU 原始数据块 — 24 字节, 6 个 float (小端)
struct ImuRawData {
    float gyroX, gyroY, gyroZ;       // 角速度 (rad/s)
    float accelX, accelY, accelZ;    // 加速度 (g)
};
static_assert(sizeof(ImuRawData) == 24);

// 控制指令包头 — 24 字节, 用于 sendCommand 构造 REQ 和解析 ACK
struct CommandHeader {
    uint8_t  sof;                    // 起始标志 (0xAA)
    uint8_t  version;                // 协议版本
    uint16_t length;                 // 总长度 (header + data)
    uint32_t seqNum;                 // 序列号 (递增)
    uint16_t cmdId;                  // 指令 ID (0x0000~0x0202)
    uint8_t  cmdType;                // 0=REQ, 1=ACK
    uint8_t  senderType;             // 0=Host, 1=Radar
    uint8_t  reserved[6];            // 保留
    uint16_t crc16;                  // CRC-16 校验 (覆盖前 18 字节)
    uint32_t crc32;                  // CRC-32 校验 (覆盖 data 段)
};
static_assert(sizeof(CommandHeader) == 24);

#pragma pack(pop)

// ────────────────────────────────────────────────────────────────────────────
//  CRC 查表算法 — 编译期生成 256 项查找表, 消除逐 bit 循环
// ────────────────────────────────────────────────────────────────────────────

// CRC-16 CCITT (XMODEM 变体), 多项式 0x1021, 初始值 0xFFFF
constexpr std::array<uint16_t, 256> kCrc16Table = []() {
    std::array<uint16_t, 256> table{};
    for (int i = 0; i < 256; ++i) {
        uint16_t crc = static_cast<uint16_t>(i) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                 : static_cast<uint16_t>(crc << 1);
        table[i] = crc;
    }
    return table;
}();

// CRC-32 反射 (IEEE 802.3), 多项式 0xEDB88320, 初始/终值 0xFFFFFFFF
constexpr std::array<uint32_t, 256> kCrc32Table = []() {
    std::array<uint32_t, 256> table{};
    for (int i = 0; i < 256; ++i) {
        uint32_t crc = static_cast<uint32_t>(i);
        for (int b = 0; b < 8; ++b)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
        table[i] = crc;
    }
    return table;
}();

// 调用方将 data 指针和 length 传入, 返回 CRC-16 校验值
inline uint16_t crc16Ccitt(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i)
        crc = static_cast<uint16_t>((crc << 8) ^ kCrc16Table[(crc >> 8) ^ data[i]]);
    return crc;
}

// 调用方将 data 指针和 length 传入, 返回 CRC-32 校验值
inline uint32_t crc32Reflect(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; ++i)
        crc = (crc >> 8) ^ kCrc32Table[(crc ^ data[i]) & 0xFF];
    return crc ^ 0xFFFFFFFF;
}

// ────────────────────────────────────────────────────────────────────────────
//  内部数据结构
// ────────────────────────────────────────────────────────────────────────────

// 解析后的单个点 — 坐标已转为 SI 单位 (m, 秒)
// 尺寸 = 32B (double 8 + float*4 16 + uint8_t 1 → 对齐到 8B)
struct Point {
    double  timestamp;     // 绝对时间 (s)
    float   x, y, z;       // 直角坐标 (m)
    float   intensity;     // 反射率 (0~255)
    uint8_t tag;           // 原始 tag 字段
};

// 单条 IMU 消息 — 陀螺/加速度计 6 通道
// 尺寸 = 32B (double 8 + float*6 24)
struct ImuMessage {
    double timestamp;      // 绝对时间 (s)
    float  angularVelocityX, angularVelocityY, angularVelocityZ;    // rad/s
    float  linearAccelX,  linearAccelY,  linearAccelZ;             // g
};

// 控制指令 ACK 解析结果
struct AckResult {
    bool     isValid    = false;      // ACK 是否通过所有校验
    uint16_t cmdId      = 0;          // 回显的指令 ID
    uint8_t  returnCode = 0xFF;       // 返回码 (RetCode)
    uint16_t errorKey   = 0;          // 出错时指向的问题 Key
    std::vector<uint8_t> payload;     // ACK payload 原始数据
};

// 球坐标 theta → (sin, cos) 缓存条目, 避免重复触发三角函数
struct ThetaTrig {
    uint16_t theta;     // 仰角原始值 (0.01°)
    float    sinVal;    // sin(theta)
    float    cosVal;    // cos(theta)
};

// ────────────────────────────────────────────────────────────────────────────
//  Mid360Driver — UDP 核心驱动
// ────────────────────────────────────────────────────────────────────────────
//  职责:
//    • 在 hostIp_ 上绑定 PCL/IMU 两个 UDP 端口 (构造函数)
//    • 通过 asio 协程异步接收 PCL/IMU 数据 → 解析 → 回调上层
//    • 提供控制命令接口 (discover, writeParams, queryParams, setWorkMode 等)
//    • 单雷达场景下仅需一个实例; 支持多雷达需多实例 + 不同端口
//
//  线程模型:
//    • 构造函数在调用方线程同步执行 (bind + co_spawn)
//    • io 线程 (外部 io_context::run()) 调度 receivePointCloud / receiveImu 协程
//    • sendCommand 在调用方线程同步阻塞, 内部使用临时 io_context 隔离

class Mid360Driver {
public:
    // 回调签名: 点云 (转移所有权, 零拷贝), IMU (常引用)
    using PointCloudCallback = std::function<void(std::vector<Point>)>;
    using ImuCallback        = std::function<void(const ImuMessage&)>;

    // 构造 → 绑定 socket → 启动两个 asio 接收协程
    // 参数 io 必须与外部 io_context 共享, 所有协程在该上下文中调度
    Mid360Driver(asio::io_context& io,
                 const std::string& hostIp,
                 uint16_t pclPort,
                 uint16_t imuPort,
                 PointCloudCallback pclCallback,
                 ImuCallback imuCallback);
    bool isPtpSynced() const { return ptpSynced_.load(); }
    ~Mid360Driver();

    Mid360Driver(const Mid360Driver&)            = delete;
    Mid360Driver& operator=(const Mid360Driver&) = delete;
    Mid360Driver(Mid360Driver&&)                  = delete;
    Mid360Driver& operator=(Mid360Driver&&)       = delete;

    // 关闭 socket → 协程自然退出 → 准备析构
    void stop();

    // ── 设备发现 ──
    // 通过广播 0x0000 发现局域网内雷达, 提取 SN / IP / 控制端口
    // targetIp 参数当前未使用 (始终广播); 保留以供未来单播发现扩展
    bool discover(const std::string& targetIp,
                  std::string& outSn,
                  std::string& outIp,
                  uint16_t& outPort,
                  int timeoutMs = 2000);

    // 设置控制通道的目标地址 (discover 成功后或直连模式下调用)
    void setRadarTarget(const std::string& ip,
                        uint16_t port = kPortControlRadar);

    // ── 参数读写 ──
    // writeParams: 批量写入参数 (Key-Value 对) → 返回是否成功
    // queryParams: 批量查询参数 → 填充 outResults
    bool writeParams(const std::vector<std::pair<uint16_t, std::vector<uint8_t>>>& kvPairs,
                     int timeoutMs = 2000);
    bool queryParams(const std::vector<uint16_t>& keys,
                     std::vector<std::pair<uint16_t, std::vector<uint8_t>>>& outResults,
                     int timeoutMs = 2000);

    // ── 便捷封装 ──
    // 每个方法内部调用 writeParams / queryParams, 封装常见的单参数操作
    bool setWorkMode(uint8_t mode, int timeoutMs = 2000);
    bool queryWorkState(uint8_t& outState, int timeoutMs = 2000);
    bool setImuEnabled(bool enabled, int timeoutMs = 2000);
    bool setDetectMode(uint8_t mode, int timeoutMs = 2000);
    bool setPclDataType(uint8_t type, int timeoutMs = 2000);

    // 设置单个推送目标的 IP+Port (payload: 4B IP + 2B port + 2B reserved)
    bool writePushDest(uint16_t key, uint16_t port, int timeoutMs = 2000);

private:
    // ── 异步接收协程 (仅 io 线程) ──
    asio::awaitable<void> receivePointCloud();   // 循环接收 PCL 数据 → parsePointCloud
    asio::awaitable<void> receiveImu();          // 循环接收 IMU 数据 → parseImu

    // ── 解析函数 (仅 io 线程) ──
    void parsePointCloud(const uint8_t* buffer, size_t length, const asio::ip::address& sender);
    void parseImu(const uint8_t* buffer, size_t length, const asio::ip::address& sender);

    // 时间戳锚定: 首次收到的雷达时间与 host 时钟建立 delta, 后续复用
    double getTimestamp(const DataHeader& header, const asio::ip::address& sender);

    // 控制指令发送与 ACK 等待 (调用方线程, 阻塞式, 内部使用独立 io_context)
    bool sendCommand(uint16_t cmdId, const uint8_t* data, size_t dataLength,
                     uint32_t seqNum, AckResult& outResult, int timeoutMs);

    // ── 成员 ──
    asio::ip::address_v4  hostIp_;           // 绑定 IP
    asio::ip::udp::socket pclSocket_;        // 点云接收 socket
    asio::ip::udp::socket imuSocket_;        // IMU 接收 socket
    PointCloudCallback    pclCallback_;      // 点云数据回调
    ImuCallback           imuCallback_;      // IMU 数据回调
    std::atomic<bool>     ptpSynced_{false};   // PTP 时间同步锁定后置 true
    std::atomic<bool>     isRunning_{true};  // 协程运行标志 (stop() → false)

    // 控制通道目标 (单播发现后已知目标的 IP:Port)
    asio::ip::address_v4  radarAddr_      = asio::ip::address_v4::broadcast();
    uint16_t              radarCtrlPort_  = kPortControlRadar;
    bool                  hasRadarAddr_   = false;

    // 时间锚定 (单雷达: 初始化时计算一次 delta, 后续每包直接加)
    double            deltaValue_  = 0.0;
    bool              hasDelta_    = false;
    asio::ip::address deltaSender_;

    // 球坐标 theta → (sin,cos) 缓存 (容量 64, 覆盖 Mid-360 的 ~40 线)
    std::array<ThetaTrig, kThetaCacheCapacity> thetaCache_;
    size_t thetaCacheSize_ = 0;

    // 接收/解析缓冲区
    std::vector<Point>                pointBuffer_;  // 单包解析结果, 所有权转移至回调
    std::array<uint8_t, kUdpBufSize> pclRxBuffer_;  // PCL UDP 接收 buffer
    std::array<uint8_t, kUdpBufSize> imuRxBuffer_;  // IMU UDP 接收 buffer

    // 调试计数器
    std::atomic<uint64_t> pclPacketsRx_{0};
    std::atomic<uint64_t> pclCrcPassed_{0};

    // 命令序列号 & 锁 (确保同一时刻只有一条控制指令在发送)
    std::atomic<uint32_t> cmdSeq_{1};
    std::mutex            cmdMutex_;
};

// ────────────────────────────────────────────────────────────────────────────
//  Mid360DriverNode — ROS2 节点
// ────────────────────────────────────────────────────────────────────────────
//  职责:
//    • 读取 ROS2 参数 → 初始化 Mid360Driver → 管理启动/重连流程
//    • 双缓冲收集点云/IMU → 定时发布 PointCloud2/Imu 消息
//    • 提供 ~/enable_sampling (SetBool) 和 ~/get_work_state (Trigger) 服务
//    • 可选: 发布静态 TF (lidar → IMU), 点云空间过滤
//    • 可选: 详细 debug 日志 (JSON lines 格式, 写入时间戳子目录)
//
//  线程模型:
//    • io 线程: ioContext_.run() → asio 协程 (receivePointCloud/IMU) → 解析 → 推入双缓冲
//    • timer 线程: wall_timer 回调 publishPointCloud() / publishImuMessages()
//    • ROS executor 线程: 服务回调 + 参数回调
//    • sendCommand 创建临时 io_context (不阻塞上述任何线程)

class Mid360DriverNode : public rclcpp::Node {
public:
    explicit Mid360DriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~Mid360DriverNode() override;

private:
    // ── 启动流程 ──
    void loadParameters();              // 读取 ROS2 参数, 校验并存储
    bool tryInitDriver(uint16_t pclPort, uint16_t imuPort);  // 创建 Mid360Driver 实例
    bool scanAndBindPorts();            // 端口扫描 (56301~56350, 56401~56450)
    bool testBind(uint16_t port);       // 临时 bind 测试端口是否空闲
    bool attemptConnectAndStart();      // discover → writePushDest → setWorkMode
    void startRetryTimer();             // 启动 2 Hz 后台重连定时器
    void onStartupRetry();              // 定时器回调: 重试绑定 + 发现 + 启动
    void startPublishTimers();          // 创建 PCL/IMU 发布定时器 (socket 绑定后立即调用)
    void onStartupSuccess();            // 启动成功: 日志 "DRIVER_READY"

    // ── ROS2 服务回调 (线程安全: 带 null guard + cmdMutex_ 保护) ──
    void onSetSampling(std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void onGetWorkState(std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // ── 数据回调 (io 线程 → 推入双缓冲, 右值引用消除拷贝) ──
    void onPointCloudReceived(std::vector<Point> points);
    void onImuReceived(const ImuMessage& message);

    // ── 定时发布 (timer 线程 → 双缓冲消费 → 发布) ──
    void publishPointCloud();
    void publishImuMessages();

    // ── 点云空间过滤 (发布时逐点检查) ──
    inline bool shouldKeepPoint(float x, float y, float z) const noexcept;

    // ── 点云格式化 ──
    static inline void writePointToBuffer(uint8_t* dest, const Point& point);
    static std::vector<sensor_msgs::msg::PointField> makePointFields();
    static const std::vector<sensor_msgs::msg::PointField> kPointFields;

    // ── Debug 日志 ──
    void initDebugLog();
    void writeDebugLog(const std::string& level, const std::string& event,
                       const std::string& msg);

    // ── 成员 ──

    // asio
    asio::io_context ioContext_;
    std::thread      ioThread_;

    // 核心驱动 (nullable: 端口不可用/重连中时为空)
    std::unique_ptr<Mid360Driver> driver_;
    uint16_t pclPort_{kHostPointCloudBase};
    uint16_t imuPort_{kHostImuBase};

    // ROS2 发布/服务/定时器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr          imuPublisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setModeService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getModeService_;
    rclcpp::TimerBase::SharedPtr pclTimer_;
    rclcpp::TimerBase::SharedPtr imuTimer_;
    rclcpp::TimerBase::SharedPtr bindRetryTimer_;

    // 静态 TF (仅发一次)
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster_;

    // 双缓冲 (pclMutex_ / imuMutex_ 保护 io-thread ↔ timer-thread 交接)
    std::mutex              pclMutex_;
    std::vector<Point>      pclBufferA_;
    std::vector<Point>      pclBufferB_;
    std::mutex              imuMutex_;
    std::vector<ImuMessage> imuBufferA_;
    bool ptpLogged_ = false;                     // PTP 同步日志已输出（避免反复打印）
    std::vector<ImuMessage> imuBufferB_;

    // 发布输出缓冲 (复用, 零拷贝 swap 接入 ROS message)
    std::vector<uint8_t> pclOutputBuffer_;

    // ROS2 配置参数
    std::string pclTopic_     = "/livox/lidar";
    std::string imuTopic_     = "/livox/imu";
    std::string pclFrameId_   = "livox_frame";
    std::string imuFrameId_   = "imu_frame";
    std::string hostIp_       = "192.168.32.80";
    std::string targetLidarIp_ = "auto";

    uint8_t pclFormat_    = DATA_TYPE_CARTESIAN_HIGH;
    int     publishHz_    = 0;
    bool    isImuEnabled_ = true;
    uint8_t detectMode_   = 0;
    bool    isTfEnabled_  = true;

    // 点云空间过滤 (启动时预计算平方阈值, 避免每点 sqrt)
    bool  isDistFilterEnabled_ = false;
    float minDistSquared_ = -1.0f;
    float maxDistSquared_ = -1.0f;
    bool  isXFilterOn_    = false;
    bool  isYFilterOn_    = false;
    bool  isZFilterOn_    = false;
    float xMin_ = 0.0f, xMax_ = 0.0f;
    float yMin_ = 0.0f, yMax_ = 0.0f;
    float zMin_ = 0.0f, zMax_ = 0.0f;
    bool  isFilterDisabled_ = true;

    // 缓冲溢出一次性警告 (防日志风暴)
    bool pclGuardWarned_ = false;

    // Debug 日志
    bool        debugLogEnabled_ = false;
    std::string logBasePath_;
    std::string logStampDir_;
    std::ofstream debugLogFile_;
    std::mutex   debugLogMutex_;

    // 服务回调串行锁 (保护 driver_ 的并发访问)
    std::mutex cmdMutex_;
};

}  // namespace mid360_driver
