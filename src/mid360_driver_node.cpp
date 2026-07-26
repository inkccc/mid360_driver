#define _USE_MATH_DEFINES
#include "mid360_driver_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mid360_driver {

// ══════════════════════════════════════════════════════════════════════════════
//  Phi 方位角 Lookup Table (sin/cos, 0~360.00° 步长 0.01°)
// ══════════════════════════════════════════════════════════════════════════════
//  Mid-360 球坐标模式每秒需计算 ~400K 次超越函数。
//  该 LUT 占用 288 KB (36001 × 2 floats × 4B), 完全消除 CPU 瓶颈。
struct PhiLut {
    std::array<float, kPhiLutSize> sin;
    std::array<float, kPhiLutSize> cos;
    PhiLut() {
        for (size_t i = 0; i < kPhiLutSize; ++i) {
            double rad = static_cast<double>(i) * 1e-2 * M_PI / 180.0;
            sin[i] = static_cast<float>(std::sin(rad));
            cos[i] = static_cast<float>(std::cos(rad));
        }
    }
};
static const PhiLut kPhiLut;

// ══════════════════════════════════════════════════════════════════════════════
//  Mid360Driver 实现
// ══════════════════════════════════════════════════════════════════════════════

// ── 构造函数 ──
// 打开并绑定 PCL/IMU 两个 UDP socket → 启动 asio 接收协程
// 参数 hostIp 必须是本机已有网卡的 IPv4 地址, 否则抛出异常
// 协程的生命周期由外部 io_context::run() 管理
Mid360Driver::Mid360Driver(asio::io_context& io,
                           const std::string& hostIp,
                           uint16_t pclPort,
                           uint16_t imuPort,
                           PointCloudCallback pclCallback,
                           ImuCallback imuCallback)
    : pclSocket_(io)
    , imuSocket_(io)
    , pclCallback_(std::move(pclCallback))
    , imuCallback_(std::move(imuCallback)) {

    std::error_code errorCode;
    hostIp_ = asio::ip::make_address_v4(hostIp, errorCode);
    if (errorCode) throw std::runtime_error("invalid IPv4 host: " + hostIp);

    pclSocket_.open(asio::ip::udp::v4(), errorCode);
    if (errorCode) throw std::runtime_error("open pcl socket: " + errorCode.message());
    pclSocket_.set_option(asio::ip::udp::socket::reuse_address(true), errorCode);
    pclSocket_.bind(asio::ip::udp::endpoint(hostIp_, pclPort), errorCode);
    if (errorCode) throw std::runtime_error("bind pcl " + hostIp + ":" +
                                            std::to_string(pclPort) + ": " + errorCode.message());

    imuSocket_.open(asio::ip::udp::v4(), errorCode);
    if (errorCode) throw std::runtime_error("open imu socket: " + errorCode.message());
    imuSocket_.set_option(asio::ip::udp::socket::reuse_address(true), errorCode);
    imuSocket_.bind(asio::ip::udp::endpoint(hostIp_, imuPort), errorCode);
    if (errorCode) throw std::runtime_error("bind imu " + hostIp + ":" +
                                            std::to_string(imuPort) + ": " + errorCode.message());

    pointBuffer_.reserve(kPointBufferReserve);

    // co_spawn documentation: posts the coroutine start to the io_context.
    // The coroutine begins execution only when io_context::run() is called.
    asio::co_spawn(io, receivePointCloud(), asio::detached);
    asio::co_spawn(io, receiveImu(),        asio::detached);
}

// ── 析构: 先关 socket 清理协程, 再等线程退出 ──
Mid360Driver::~Mid360Driver() { stop(); }

// ── stop ──
// 设置 isRunning_=false → 关闭 socket → 正在进行的 async_receive_from 返回错误
// → 协程检查 isRunning_ 后退出循环
void Mid360Driver::stop() {
    isRunning_.store(false, std::memory_order_relaxed);
    asio::error_code errorCode;
    pclSocket_.close(errorCode);
    imuSocket_.close(errorCode);
}

// ── setRadarTarget ──
// 记录已发现雷达的 IP 和控制端口, 供 sendCommand 单播使用
// discover 成功后或直连模式下调用
void Mid360Driver::setRadarTarget(const std::string& ip, uint16_t port) {
    std::error_code errorCode;
    auto v4 = asio::ip::make_address_v4(ip, errorCode);
    if (!errorCode) {
        radarAddr_     = v4;
        radarCtrlPort_ = port;
        hasRadarAddr_  = true;
    }
}

// ── getTimestamp ──
// 将雷达原始时间戳 (ns) 转换为 host 时间。
//   TIMESTAMP_NO_SYNC: 首次到达 → 计算 delta = hostClock - radarClock
//                      后续到达 → radarTs + delta, 复用已缓存的 delta
//   TIMESTAMP_PTP/GPS: 直接返回 (已经同步过)
// 调用线程: io 线程 (协程上下文), 无并发风险
double Mid360Driver::getTimestamp(const DataHeader& header,
                                   const asio::ip::address& sender) {
    double timestampSec = static_cast<double>(header.timestamp) * 1e-9;
    if (header.timeType != TIMESTAMP_NO_SYNC) { ptpSynced_ = true; return timestampSec; }

    if (!hasDelta_) {
        auto now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        deltaValue_  = now - timestampSec;
        deltaSender_ = sender;
        hasDelta_    = true;
        return now;
    }
    if (sender == deltaSender_) [[likely]]
        return timestampSec + deltaValue_;

    // 雷达 IP 变化 (罕见: 替换设备或 DHCP 换 IP) → 重新锚定
    auto now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    deltaValue_  = now - timestampSec;
    deltaSender_ = sender;
    return now;
}

// ── receivePointCloud ──
// asio 协程, 在 io 线程上调度。
// 循环: async_receive_from → 验证源端口和数据长度 → parsePointCloud
// 异常或在 stop() 后 socket 关闭时退出 (由 isRunning_ 标记保证)
asio::awaitable<void> Mid360Driver::receivePointCloud() {
    asio::ip::udp::endpoint senderEndpoint;
    try {
        while (isRunning_.load(std::memory_order_relaxed)) {
            asio::error_code errorCode;
            auto received = co_await pclSocket_.async_receive_from(
                asio::buffer(pclRxBuffer_), senderEndpoint,
                asio::redirect_error(asio::use_awaitable, errorCode));
            if (errorCode) [[unlikely]] continue;
            if (senderEndpoint.port() != kPortPointCloudRadar
                || received < sizeof(DataHeader)) [[unlikely]] continue;
            pclPacketsRx_.fetch_add(1, std::memory_order_relaxed);
            parsePointCloud(pclRxBuffer_.data(), received, senderEndpoint.address());
        }
    } catch (const std::exception& e) {
        std::cerr << "[PCL coroutine] exception: " << e.what() << std::endl;
    }
    co_return;
}

// ── receiveImu ──
// 点云协程的 IMU 版本。校验源端口为 56400 + 数据长度 ≥ DataHeader + ImuRawData
asio::awaitable<void> Mid360Driver::receiveImu() {
    asio::ip::udp::endpoint senderEndpoint;
    try {
        while (isRunning_.load(std::memory_order_relaxed)) {
            asio::error_code errorCode;
            auto received = co_await imuSocket_.async_receive_from(
                asio::buffer(imuRxBuffer_), senderEndpoint,
                asio::redirect_error(asio::use_awaitable, errorCode));
            if (errorCode) [[unlikely]] continue;
            if (senderEndpoint.port() != kPortImuRadar
                || received < sizeof(DataHeader) + sizeof(ImuRawData)) [[unlikely]] continue;
            parseImu(imuRxBuffer_.data(), received, senderEndpoint.address());
        }
    } catch (const std::exception& e) {}
    co_return;
}

// ── parsePointCloud ──
// 解析一个 UDP 数据包中的点云数据。
// 流程:
//   读取 DataHeader → CRC-32 校验 → 计算 baseTimestamp 和 perPointDelta
//   → 按 dataType 分派到三种格式的解析 → tag 过滤 (0xBF: 噪声|低置信度)
//   → 调用 pclCallback_(std::move(pointBuffer_)) 转移所有权 (零拷贝)
// 球坐标模式: 使用 theta 缓存 (线性扫描, 容量 64) 将 4 次 trig/点降为 2 次/点
void Mid360Driver::parsePointCloud(const uint8_t* buffer, size_t length,
                                    const asio::ip::address& sender) {
    DataHeader header;
    std::memcpy(&header, buffer, sizeof(header));
    if (!header.dotNum) return;

    // CRC-32 完整性校验 (覆盖 data 段)
    // Mid-360 默认不填充 CRC-32 (字段为 0); 仅在雷达手动开启 CRC 时才校验
    size_t dataLen = length - sizeof(DataHeader);
    if (dataLen && header.crc32 != 0
        && crc32Reflect(buffer + sizeof(DataHeader), dataLen) != header.crc32)
        return;
    pclCrcPassed_.fetch_add(1, std::memory_order_relaxed);

    const double baseTimestamp = getTimestamp(header, sender);
    // timeInterval 为整包时间 (0.1μs), 除以 dotNum 得每点线性插值步长
    const double perPointDelta = static_cast<double>(header.timeInterval)
                               * 1e-7 / static_cast<double>(header.dotNum);
    pointBuffer_.clear();

    const uint8_t* pointData  = buffer + sizeof(DataHeader);
    const size_t   dataLength = length - sizeof(DataHeader);

    switch (header.dataType) {

    // 直角坐标高精度 (int32 mm) → 乘以 1e-3 转为 m
    case DATA_TYPE_CARTESIAN_HIGH: {
        if (dataLength < size_t(header.dotNum) * sizeof(CartesianHighPoint))
            return;
        const auto* raw = reinterpret_cast<const CartesianHighPoint*>(pointData);
        for (uint16_t i = 0; i < header.dotNum; ++i) {
            const auto& rp = raw[i];
            if (rp.tag & 0xBFu) continue;  // 噪声 (bit7) | 低置信度 (bit0-5)
            pointBuffer_.push_back({
                baseTimestamp + perPointDelta * i,
                static_cast<float>(rp.x) * 1e-3f,
                static_cast<float>(rp.y) * 1e-3f,
                static_cast<float>(rp.z) * 1e-3f,
                static_cast<float>(rp.reflectivity),
                rp.tag});
        }
        break;
    }

    // 直角坐标低精度 (int16 10mm) → 乘以 1e-2 转为 m
    case DATA_TYPE_CARTESIAN_LOW: {
        if (dataLength < size_t(header.dotNum) * sizeof(CartesianLowPoint))
            return;
        const auto* raw = reinterpret_cast<const CartesianLowPoint*>(pointData);
        for (uint16_t i = 0; i < header.dotNum; ++i) {
            const auto& rp = raw[i];
            if (rp.tag & 0xBFu) continue;
            pointBuffer_.push_back({
                baseTimestamp + perPointDelta * i,
                static_cast<float>(rp.x) * 1e-2f,
                static_cast<float>(rp.y) * 1e-2f,
                static_cast<float>(rp.z) * 1e-2f,
                static_cast<float>(rp.reflectivity),
                rp.tag});
        }
        break;
    }

    // 球坐标 (depth mm + theta/phi 0.01°) → 转为直角坐标
    // theta = 仰角 (0=+Z), phi = 方位角 (XY平面)
    // x = r·sin(θ)·cos(φ),  y = r·sin(θ)·sin(φ),  z = r·cos(θ)
    // phi 使用预计算 LUT (36001 entries, 288KB), 完全消除超越函数开销
    case DATA_TYPE_SPHERICAL: {
        if (dataLength < size_t(header.dotNum) * sizeof(SphericalPoint))
            return;
        const auto* raw = reinterpret_cast<const SphericalPoint*>(pointData);
        for (uint16_t i = 0; i < header.dotNum; ++i) {
            const auto& sp = raw[i];
            if (sp.tag & 0xBFu) continue;

            float sinElev, cosElev;
            bool found = false;
            for (size_t j = 0; j < thetaCacheSize_; ++j) {
                if (thetaCache_[j].theta == sp.theta) [[likely]] {
                    sinElev = thetaCache_[j].sinVal;
                    cosElev = thetaCache_[j].cosVal;
                    found   = true;
                    break;
                }
            }
            if (!found) {
                double rad = sp.theta * 1e-2 * M_PI / 180.0;
                sinElev = static_cast<float>(std::sin(rad));
                cosElev = static_cast<float>(std::cos(rad));
                if (thetaCacheSize_ < kThetaCacheCapacity) {
                    thetaCache_[thetaCacheSize_] = {sp.theta, sinElev, cosElev};
                    ++thetaCacheSize_;
                }
            }
            // phi 查 LUT (clamp 到 [0, 36000])
            uint16_t phiIdx = (sp.phi <= 36000) ? sp.phi : uint16_t(0);
            float phiSin = kPhiLut.sin[phiIdx];
            float phiCos = kPhiLut.cos[phiIdx];
            float radius = static_cast<float>(sp.depth) * 1e-3f;
            pointBuffer_.push_back({
                baseTimestamp + perPointDelta * i,
                radius * sinElev * phiCos,
                radius * sinElev * phiSin,
                radius * cosElev,
                static_cast<float>(sp.reflectivity),
                sp.tag});
        }
        break;
    }

    default: return;
    }

    // 将解析结果所有权通过 move 转移给上层回调 (消除一次完整 vector 拷贝)
    // pointBuffer_ 在 move 后处于合法但未定义状态 → clear + reserve 复用
    if (!pointBuffer_.empty() && pclCallback_) {
        pclCallback_(std::move(pointBuffer_));
        pointBuffer_.clear();
        pointBuffer_.reserve(kPointBufferReserve);
    }
}

// ── parseImu ──
// 解析一个 IMU UDP 数据包。
// CRC-32 校验 → memcpy ImuRawData → 回调上层 (常引用, 零拷贝)
void Mid360Driver::parseImu(const uint8_t* buffer, size_t length,
                             const asio::ip::address& sender) {
    DataHeader header;
    std::memcpy(&header, buffer, sizeof(header));
    ImuRawData raw;
    std::memcpy(&raw, buffer + sizeof(DataHeader), sizeof(raw));
    if (imuCallback_) imuCallback_({
        getTimestamp(header, sender),
        raw.gyroX,  raw.gyroY,  raw.gyroZ,
        raw.accelX, raw.accelY, raw.accelZ});
}

// ── sendCommand ──
// 控制指令发送 & ACK 接收 (阻塞式)。
// 流程:
//   构造 CommandHeader (填充 sof/version/length/seqNum/cmdId → CRC-16)
//   → 填充 data 段 → CRC-32 → 发送到目标 IP:Port
//   → 创建独立临时 socket (避免影响主 io_context)
//   → 非阻塞轮询接收 ACK (2ms 间隔)
//   → 验证 sof/seqNum/cmdId/cmdType/senderType → CRC-16 头部 → CRC-32 payload
//   → 通过 → 填充 AckResult; 超时 → 返回 false
//
// 线程安全: cmdMutex_ 确保同一时刻只有一条控制指令在执行
bool Mid360Driver::sendCommand(uint16_t cmdId, const uint8_t* data,
                                size_t dataLength, uint32_t seqNum,
                                AckResult& outResult, int timeoutMs) {
    if (dataLength > kCmdDataMaxLen) [[unlikely]] return false;
    std::lock_guard<std::mutex> lock(cmdMutex_);

    // 独立 io_context + socket, 隔离控制通道与数据通道
    asio::io_context tempIo;
    asio::ip::udp::socket socket(tempIo);
    asio::error_code errorCode;
    socket.open(asio::ip::udp::v4(), errorCode);
    if (errorCode) return false;

    // 构建 CommandHeader (24 字节固定头)
    uint8_t  packetBuf[kCmdHeaderSize + kCmdDataMaxLen];
    uint16_t totalLength = static_cast<uint16_t>(kCmdHeaderSize + dataLength);
    auto&    cmdHeader   = *reinterpret_cast<CommandHeader*>(packetBuf);
    cmdHeader.sof        = kCmdSof;
    cmdHeader.version    = kCmdVersion;
    cmdHeader.length     = totalLength;
    cmdHeader.seqNum     = seqNum;
    cmdHeader.cmdId      = cmdId;
    cmdHeader.cmdType    = 0;     // REQ
    cmdHeader.senderType = 0;     // Host
    std::memset(cmdHeader.reserved, 0, 6);
    cmdHeader.crc16 = crc16Ccitt(packetBuf, 18);          // CRC-16 覆盖前 18 字节
    cmdHeader.crc32 = dataLength ? crc32Reflect(data, dataLength) : 0;  // CRC-32 覆盖 data 段
    if (dataLength) std::memcpy(packetBuf + kCmdHeaderSize, data, dataLength);

    // 选择目标: 发现指令 → 广播到 56000; 其他 → 已知雷达 (或广播) 到控制端口
    bool isDiscovery = (cmdId == CMD_DISCOVERY);
    asio::ip::udp::endpoint destination;
    if (isDiscovery) {
        socket.set_option(asio::ip::udp::socket::broadcast(true), errorCode);
        socket.bind(asio::ip::udp::endpoint(hostIp_, 0), errorCode);
        destination = asio::ip::udp::endpoint(asio::ip::address_v4::broadcast(),
                                               kPortDiscoveryRadar);
    } else {
        auto target = hasRadarAddr_ ? radarAddr_
                                    : asio::ip::address_v4::broadcast();
        destination = asio::ip::udp::endpoint(target, radarCtrlPort_);
        socket.set_option(asio::ip::udp::socket::broadcast(!hasRadarAddr_), errorCode);
        socket.bind(asio::ip::udp::endpoint(hostIp_, 0), errorCode); // 强制绑到正确网卡
    }
    socket.send_to(asio::buffer(packetBuf, totalLength), destination);

    // 切换到非阻塞模式, 轮询等待 ACK
    socket.non_blocking(true, errorCode);
    uint8_t recvBuf[kCmdHeaderSize + kCmdDataMaxLen];
    asio::ip::udp::endpoint senderEndpoint;
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeoutMs);

    while (std::chrono::steady_clock::now() < deadline) {
        size_t received = socket.receive_from(
            asio::buffer(recvBuf), senderEndpoint, 0, errorCode);
        if (errorCode) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        if (received < kCmdHeaderSize) continue;

        // 逐层验证 ACK 包头的每个字段
        CommandHeader ackHeader;
        std::memcpy(&ackHeader, recvBuf, sizeof(ackHeader));
        if (ackHeader.sof != kCmdSof) continue;
        if (ackHeader.seqNum != seqNum) continue;
        if (ackHeader.cmdId != cmdId) continue;
        if (ackHeader.cmdType != 1) continue;       // ACK
        if (ackHeader.senderType != 1) continue;    // Radar

        // CRC-16 校验头
        if (crc16Ccitt(recvBuf, 18) != ackHeader.crc16) continue;

        // 安全提取 payload 长度 (防远程篡改的 length 字段)
        size_t payloadLen = (ackHeader.length > kCmdHeaderSize)
                          ? (ackHeader.length - kCmdHeaderSize) : 0;
        size_t maxPayload = (received > kCmdHeaderSize)
                          ? (received - kCmdHeaderSize) : 0;
        if (payloadLen > maxPayload) payloadLen = maxPayload;
        const uint8_t* payloadPtr = recvBuf + kCmdHeaderSize;

        // CRC-32 校验 payload
        if (payloadLen > 0) {
            if (crc32Reflect(payloadPtr, payloadLen) != ackHeader.crc32) continue;
        } else if (ackHeader.crc32 != 0) continue;

        // 全部通过 → 填充结果
        outResult.isValid = true;
        outResult.cmdId   = ackHeader.cmdId;
        if (payloadLen > 0) {
            outResult.payload.assign(payloadPtr, payloadPtr + payloadLen);
            outResult.returnCode = outResult.payload[0];
            if (outResult.payload.size() >= 3)
                outResult.errorKey = static_cast<uint16_t>(outResult.payload[1])
                                   | (static_cast<uint16_t>(outResult.payload[2]) << 8);
        }
        socket.close(errorCode);
        return true;
    }
    socket.close(errorCode);
    return false;   // 超时
}

// ── discover ──
// 发送 CMD_DISCOVERY 广播, 解析 ACK 中的 SN (16B) / IP (4B 网络序) / Port (2B 小端)
// 调用方线程执行, 阻塞 timeoutMs
bool Mid360Driver::discover(const std::string& /*targetIp*/,
                             std::string& outSn, std::string& outIp,
                             uint16_t& outPort, int timeoutMs) {
    AckResult ackResult;
    uint32_t seqNum = cmdSeq_.fetch_add(1, std::memory_order_relaxed);
    if (!sendCommand(CMD_DISCOVERY, nullptr, 0, seqNum, ackResult, timeoutMs)
        || !ackResult.isValid)
        return false;
    if (ackResult.payload.size() < 24
        || ackResult.payload[0] != RET_SUCCESS)
        return false;

    // SN: byte[2..17] → C-string → 截断到 null
    outSn.assign(reinterpret_cast<const char*>(&ackResult.payload[2]), 16);
    auto nullPos = outSn.find('\0');
    if (nullPos != std::string::npos) outSn.resize(nullPos);

    // IP: byte[18..21] 逐字节拼接 (网络字节序 = 大端)
    outIp = std::to_string(ackResult.payload[18]) + "."
          + std::to_string(ackResult.payload[19]) + "."
          + std::to_string(ackResult.payload[20]) + "."
          + std::to_string(ackResult.payload[21]);

    // 控制端口: byte[22..23] 小端 uint16
    outPort = static_cast<uint16_t>(ackResult.payload[22])
            | (static_cast<uint16_t>(ackResult.payload[23]) << 8);
    return true;
}

// ── writeParams ──
// 构造 CMD_PARAM_CONFIG 请求 payload: [keyCount LE][reserved 2B][key LE+valueLen LE+value]*N
// → sendCommand → 检查返回码 (RET_SUCCESS 或 RET_REBOOT_OK 都算成功)
bool Mid360Driver::writeParams(
    const std::vector<std::pair<uint16_t, std::vector<uint8_t>>>& kvPairs,
    int timeoutMs) {
    std::vector<uint8_t> buffer;
    buffer.reserve(4 + kvPairs.size() * 8);
    uint16_t keyCount = static_cast<uint16_t>(kvPairs.size());
    buffer.insert(buffer.end(), {static_cast<uint8_t>(keyCount & 0xFF),
                                  static_cast<uint8_t>(keyCount >> 8),
                                  0, 0});
    for (auto& [key, value] : kvPairs) {
        uint16_t valueLen = static_cast<uint16_t>(value.size());
        buffer.insert(buffer.end(), {static_cast<uint8_t>(key & 0xFF),
                                      static_cast<uint8_t>(key >> 8),
                                      static_cast<uint8_t>(valueLen & 0xFF),
                                      static_cast<uint8_t>(valueLen >> 8)});
        buffer.insert(buffer.end(), value.begin(), value.end());
    }
    AckResult ackResult;
    uint32_t seqNum = cmdSeq_.fetch_add(1, std::memory_order_relaxed);
    if (!sendCommand(CMD_PARAM_CONFIG, buffer.data(), buffer.size(),
                     seqNum, ackResult, timeoutMs) || !ackResult.isValid)
        return false;
    return ackResult.returnCode == RET_SUCCESS
        || ackResult.returnCode == RET_REBOOT_OK;
}

// ── queryParams ──
// 构造 CMD_PARAM_QUERY 请求 payload → sendCommand → 解析出 key-value 列表
// 返回的每个键值对通过边界检查保护, 防止 payload 截断导致的越界读
bool Mid360Driver::queryParams(const std::vector<uint16_t>& keys,
                               std::vector<std::pair<uint16_t, std::vector<uint8_t>>>& outResults,
                               int timeoutMs) {
    std::vector<uint8_t> buffer;
    uint16_t keyCount = static_cast<uint16_t>(keys.size());
    buffer.insert(buffer.end(), {static_cast<uint8_t>(keyCount & 0xFF),
                                  static_cast<uint8_t>(keyCount >> 8),
                                  0, 0});
    for (auto key : keys)
        buffer.insert(buffer.end(), {static_cast<uint8_t>(key & 0xFF),
                                      static_cast<uint8_t>(key >> 8)});

    AckResult ackResult;
    uint32_t seqNum = cmdSeq_.fetch_add(1, std::memory_order_relaxed);
    if (!sendCommand(CMD_PARAM_QUERY, buffer.data(), buffer.size(),
                     seqNum, ackResult, timeoutMs) || !ackResult.isValid)
        return false;
    if (ackResult.payload.size() < 3
        || ackResult.payload[0] != RET_SUCCESS)
        return false;

    uint16_t resultKeyCount = static_cast<uint16_t>(ackResult.payload[1])
                            | (static_cast<uint16_t>(ackResult.payload[2]) << 8);
    size_t offset = 3;
    for (uint16_t i = 0; i < resultKeyCount; ++i) {
        if (offset + 4 > ackResult.payload.size()) break;
        uint16_t key      = static_cast<uint16_t>(ackResult.payload[offset])
                          | (static_cast<uint16_t>(ackResult.payload[offset + 1]) << 8);
        uint16_t valueLen = static_cast<uint16_t>(ackResult.payload[offset + 2])
                          | (static_cast<uint16_t>(ackResult.payload[offset + 3]) << 8);
        if (offset + 4 + valueLen > ackResult.payload.size()) break;
        outResults.emplace_back(key,
            std::vector<uint8_t>(ackResult.payload.begin() + offset + 4,
                                  ackResult.payload.begin() + offset + 4 + valueLen));
        offset += 4 + valueLen;
    }
    return true;
}

// ── 便捷封装 (薄层, 全部委托给 writeParams / queryParams) ──

bool Mid360Driver::setWorkMode(uint8_t mode, int timeoutMs) {
    return writeParams({{KEY_WORK_TGT_MODE, {mode}}}, timeoutMs);
}
bool Mid360Driver::queryWorkState(uint8_t& outState, int timeoutMs) {
    std::vector<std::pair<uint16_t, std::vector<uint8_t>>> results;
    if (!queryParams({KEY_CUR_WORK_STATE}, results, timeoutMs)
        || results.empty() || results[0].second.empty())
        return false;
    outState = results[0].second[0];
    return true;
}
bool Mid360Driver::setImuEnabled(bool enabled, int timeoutMs) {
    return writeParams({{KEY_IMU_DATA_EN,
        {static_cast<uint8_t>(enabled ? 1 : 0)}}}, timeoutMs);
}
bool Mid360Driver::setDetectMode(uint8_t mode, int timeoutMs) {
    return writeParams({{KEY_DETECT_MODE, {mode}}}, timeoutMs);
}
bool Mid360Driver::setPclDataType(uint8_t type, int timeoutMs) {
    return writeParams({{KEY_PCL_DATA_TYPE, {type}}}, timeoutMs);
}

// ── writePushDest ──
// 设置单个推送目标的 IP+Port
// payload: 4B IP + 2B port (LE) + 2B reserved = 8 字节
bool Mid360Driver::writePushDest(uint16_t key, uint16_t port, int timeoutMs) {
    auto ipBytes = hostIp_.to_bytes();
    std::vector<uint8_t> val = {
        ipBytes[0], ipBytes[1], ipBytes[2], ipBytes[3],
        static_cast<uint8_t>(port & 0xFF),
        static_cast<uint8_t>((port >> 8) & 0xFF),
        0, 0};
    return writeParams({{key, val}}, timeoutMs);
}

// ══════════════════════════════════════════════════════════════════════════════
//  Mid360DriverNode 实现
// ══════════════════════════════════════════════════════════════════════════════

// ── kPointFields (静态) ──
// PointCloud2 字段定义: x,y,z (FLOAT32) + intensity (FLOAT32)
//                       + timestamp (FLOAT64) + tag (UINT8) = 25 字节/点
const std::vector<sensor_msgs::msg::PointField> Mid360DriverNode::kPointFields =
    Mid360DriverNode::makePointFields();

std::vector<sensor_msgs::msg::PointField> Mid360DriverNode::makePointFields() {
    std::vector<sensor_msgs::msg::PointField> fields;
    fields.reserve(6);
    sensor_msgs::msg::PointField field;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count    = 1;

    field.name = "x";          field.offset = 0;  fields.push_back(field);
    field.name = "y";          field.offset = 4;  fields.push_back(field);
    field.name = "z";          field.offset = 8;  fields.push_back(field);
    field.name = "intensity";  field.offset = 12; fields.push_back(field);

    field.datatype = sensor_msgs::msg::PointField::FLOAT64;
    field.name     = "timestamp";
    field.offset   = 16;
    fields.push_back(field);

    field.datatype = sensor_msgs::msg::PointField::UINT8;
    field.name     = "tag";
    field.offset   = 24;
    fields.push_back(field);

    return fields;
}

// ── 构造函数 ──
// 启动流程:
//   loadParameters → 创建发布/服务 → 发 TF (可选)
//   → tryInitDriver (默认端口) → scanAndBindPorts (扫描空闲端口)
//   → 绑定成功 → 启动 io 线程 → attemptConnectAndStart
//     → 成功 → onStartupSuccess (创建发布定时器)
//     → 失败 → startRetryTimer (2 Hz 后台重连)
//   → 绑定失败 → startRetryTimer (2 Hz 后台重试绑定)
Mid360DriverNode::Mid360DriverNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mid360_driver_node", options) {

    RCLCPP_INFO(get_logger(), "========== Mid-360 Driver Start ==========");
    loadParameters();
    initDebugLog();
    writeDebugLog("INFO", "DRIVER_START", "node constructed");

    pclPublisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(pclTopic_, 1000);
    imuPublisher_ = create_publisher<sensor_msgs::msg::Imu>(imuTopic_, 1000);
    pclBufferA_.reserve(kMaxBufferPoints);
    pclBufferB_.reserve(kMaxBufferPoints);
    imuBufferA_.reserve(256);
    imuBufferB_.reserve(256);

    setModeService_ = create_service<std_srvs::srv::SetBool>(
        "~/enable_sampling",
        [this](std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            onSetSampling(request, response);
        });
    getModeService_ = create_service<std_srvs::srv::Trigger>(
        "~/get_work_state",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            onGetWorkState(request, response);
        });

    writeDebugLog("INFO", "ROS_INIT", "publishers & services created");

    // 静态 TF (IMU 在雷达坐标系中的位姿)
    if (isTfEnabled_) {
        staticTfBroadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp    = now();
        transform.header.frame_id = pclFrameId_;
        transform.child_frame_id  = imuFrameId_;
        transform.transform.translation.x = kImuOffsetX;
        transform.transform.translation.y = kImuOffsetY;
        transform.transform.translation.z = kImuOffsetZ;
        transform.transform.rotation.w    = 1.0;
        staticTfBroadcaster_->sendTransform(transform);
    }

    // 尝试绑定 → 失败则扫描空闲端口
    bool bound = tryInitDriver(pclPort_, imuPort_);
    if (!bound) {
        bound = scanAndBindPorts();
    }

    if (bound) {
        writeDebugLog("INFO", "SOCKET_BOUND",
                      std::string("pcl=") + std::to_string(pclPort_) +
                      " imu=" + std::to_string(imuPort_));
        ioThread_ = std::thread([this] { ioContext_.run(); });
        // 立即启动发布定时器, 避免数据在 discovered 等待期间溢出缓冲
        startPublishTimers();
        if (attemptConnectAndStart()) {
            onStartupSuccess();
        } else {
            RCLCPP_WARN(get_logger(), "3s 内未发现雷达, 启动后台静默重连 (2 Hz)...");
            writeDebugLog("INFO", "RETRY_START",
                          "initial connect failed, starting 2Hz background retry");
            startRetryTimer();
        }
    } else {
        RCLCPP_WARN(get_logger(), "端口不可用, 启动后台静默重连 (2 Hz)...");
        writeDebugLog("WARN", "RETRY_START", "no available port, starting 2Hz background retry");
        startRetryTimer();
    }
}

// ── startPublishTimers ──
// 创建 PCL/IMU 发布定时器 (在 socket 绑定后立即调用, 防止缓冲溢出)。
// PCL 频率由 publishHz_ 控制 (0→1ms, N→1000/N ms)
// IMU 固定 1ms 轮询
void Mid360DriverNode::startPublishTimers() {
    int intervalMs = publishHz_ <= 0 ? 1 : (1000 / publishHz_);
    pclTimer_ = create_wall_timer(std::chrono::milliseconds(intervalMs),
                                  [this] { publishPointCloud(); });
    imuTimer_ = create_wall_timer(std::chrono::milliseconds(1),
                                  [this] { publishImuMessages(); });
}

// ── onStartupSuccess ──
// 驱动已就绪 (数据通道建立, 雷达已确认在采样模式)。
// 发布定时器在 socket 绑定时已启动, 此处仅打日志
void Mid360DriverNode::onStartupSuccess() {
    RCLCPP_INFO(get_logger(), "========== Mid-360 Driver Ready ==========");
    writeDebugLog("INFO", "DRIVER_READY",
                  std::string("publishHz=") + std::to_string(publishHz_));
}

// ── 析构函数 ──
// 关闭顺序: driver_->stop() (关闭 socket 清协程)
//          → ioContext_.stop() (停止 event loop)
//          → ioThread_.join() (等待线程退出)
Mid360DriverNode::~Mid360DriverNode() {
    writeDebugLog("INFO", "DRIVER_SHUTDOWN", "destructor called");
    if (bindRetryTimer_) bindRetryTimer_->cancel();
    if (driver_) driver_->stop();
    ioContext_.stop();
    if (ioThread_.joinable()) ioThread_.join();
}

// ── scanAndBindPorts ──
// 在 [base, base+49] 范围内扫描 PCL 和 IMU 端口, 找到第一个同时空闲的对
// 调用 testBind 临时 bind → close → tryInitDriver 正式绑定
bool Mid360DriverNode::scanAndBindPorts() {
    for (uint16_t offset = 0; offset < kPortScanLimit; ++offset) {
        uint16_t testPcl = kHostPointCloudBase + offset;
        uint16_t testImu = kHostImuBase      + offset;
        if (testBind(testPcl) && testBind(testImu)) {
            pclPort_ = testPcl; imuPort_ = testImu;
            return tryInitDriver(pclPort_, imuPort_);
        }
    }
    return false;
}

// ── testBind ──
// 临时 open + bind + close 一个端口, 测试是否空闲
// 注意: 存在 TOCTOU 竞争 (close 后 tryInitDriver 重绑时可能被抢占)
// 在嵌入式单进程环境下可接受
bool Mid360DriverNode::testBind(uint16_t port) {
    asio::error_code ec;
    asio::ip::udp::socket s(ioContext_);
    s.open(asio::ip::udp::v4(), ec); if (ec) return false;
    s.bind(asio::ip::udp::endpoint(asio::ip::make_address_v4(hostIp_), port), ec);
    s.close();
    return !ec;
}

// ── startRetryTimer ──
// 创建 500ms 间隔的 wall_timer, 静默重试绑定 & 发现
void Mid360DriverNode::startRetryTimer() {
    bindRetryTimer_ = create_wall_timer(
        std::chrono::milliseconds(kBindRetryPeriodMs),
        [this] { onStartupRetry(); });
}

// ── onStartupRetry ──
// 定时器回调 (每 500ms):
//   无 driver → tryInitDriver / scanAndBindPorts → 启动 io 线程
//   有 driver → attemptConnectAndStart → 成功 → 取消 timer + onStartupSuccess
// 注意: 重新创建 driver_ 时必须先 stop+join 旧 ioThread_, 避免 std::terminate
void Mid360DriverNode::onStartupRetry() {
    if (!driver_) {
        // 停止并 join 可能残留的旧 io 线程 (防御性)
        ioContext_.stop();
        if (ioThread_.joinable()) ioThread_.join();
        ioContext_.restart();
        if (tryInitDriver(pclPort_, imuPort_) || scanAndBindPorts()) {
            ioThread_ = std::thread([this] { ioContext_.run(); });
            if (!pclTimer_) startPublishTimers();
        } else {
            return;
        }
    }
    writeDebugLog("INFO", "RETRY_ATTEMPT", "connection retry cycle");
    if (attemptConnectAndStart()) {
        bindRetryTimer_->cancel();
        RCLCPP_INFO(get_logger(), "雷达已上线");
        writeDebugLog("INFO", "RETRY_SUCCESS", "radar connected after retry");
        onStartupSuccess();
    }
}

// ── loadParameters ──
// 从 ROS2 参数服务器读取所有配置项并校验。
// pclFormat_ / publishHz_ / detectMode_ 进行范围校验 (越界 → 默认值)
// 点云过滤阈值预计算为平方值 (避免发布时每点 sqrt)
void Mid360DriverNode::loadParameters() {
    hostIp_         = declare_parameter<std::string>("host_ip", "192.168.32.80");
    targetLidarIp_  = declare_parameter<std::string>("lidar_ip", "auto");
    pclTopic_       = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    imuTopic_       = declare_parameter<std::string>("imu_topic", "/livox/imu");
    pclFrameId_     = declare_parameter<std::string>("lidar_frame", "livox_frame");
    imuFrameId_     = declare_parameter<std::string>("imu_frame", "imu_frame");
    pclFormat_      = static_cast<uint8_t>(declare_parameter<int>("pcl_data_type", 1));
    publishHz_      = declare_parameter<int>("publish_freq_hz", 0);

    // 范围校验: publishHz_ ∈ [0, 1000]; pclFormat_ ∈ {1,2,3}
    if (publishHz_ < 0) publishHz_ = 0;
    if (publishHz_ > 1000) publishHz_ = 1000;
    if (pclFormat_ > DATA_TYPE_SPHERICAL) {
        RCLCPP_WARN(get_logger(), "invalid pcl_data_type=%d, fallback to CARTESIAN_HIGH", pclFormat_);
        pclFormat_ = DATA_TYPE_CARTESIAN_HIGH;
    }

    isImuEnabled_   = declare_parameter<bool>("imu_data_en", true);
    detectMode_     = static_cast<uint8_t>(declare_parameter<int>("detect_mode", 0));
    isTfEnabled_    = declare_parameter<bool>("publish_tf", true);

    // Debug 日志
    debugLogEnabled_ = declare_parameter<bool>("debug_log_enable", false);
    logBasePath_     = declare_parameter<std::string>("log_base_path",
        "/home/inkc/inkc-ws/v2027/src/driver/mid360_driver/log/");

    // 点云空间过滤
    float minDistance = declare_parameter<float>("min_point_distance", -1.0f);
    float maxDistance = declare_parameter<float>("max_point_distance", -1.0f);
    isXFilterOn_  = declare_parameter<bool>("enable_x_filter", false);
    xMin_ = declare_parameter<float>("min_x", -2.0f);
    xMax_ = declare_parameter<float>("max_x",  2.0f);
    isYFilterOn_  = declare_parameter<bool>("enable_y_filter", false);
    yMin_ = declare_parameter<float>("min_y", -2.0f);
    yMax_ = declare_parameter<float>("max_y",  2.0f);
    isZFilterOn_  = declare_parameter<bool>("enable_z_filter", false);
    zMin_ = declare_parameter<float>("min_z", -2.0f);
    zMax_ = declare_parameter<float>("max_z",  2.0f);

    // 距离过滤: 预存平方阈值, 发布时用 d² 比较 (避免每点 sqrt)
    isDistFilterEnabled_ = (minDistance >= 0) || (maxDistance >= 0);
    minDistSquared_ = (minDistance >= 0) ? minDistance * minDistance : -1;
    maxDistSquared_ = (maxDistance >= 0) ? maxDistance * maxDistance : -1;
    isFilterDisabled_ = !isDistFilterEnabled_ && !isXFilterOn_
                     && !isYFilterOn_ && !isZFilterOn_;

    RCLCPP_INFO(get_logger(),
        "host=%s lidar=%s fmt=%d hz=%d imu=%d detect=%d tf=%d debug=%d",
        hostIp_.c_str(), targetLidarIp_.c_str(), pclFormat_,
        publishHz_, isImuEnabled_, detectMode_, isTfEnabled_, debugLogEnabled_);

    writeDebugLog("INFO", "PARAM_LOADED",
        std::string("host=") + hostIp_ +
        " lidar=" + targetLidarIp_ +
        " fmt=" + std::to_string(pclFormat_) +
        " hz=" + std::to_string(publishHz_) +
        " imu=" + std::to_string(isImuEnabled_) +
        " detect=" + std::to_string(detectMode_) +
        " tf=" + std::to_string(isTfEnabled_) +
        " debug=" + std::to_string(debugLogEnabled_));
}

// ── tryInitDriver ──
// 用当前 pclPort_/imuPort_ 构造 Mid360Driver。
// 如端口冲突或 IP 无效 → 返回 false; 异常流程由 scanAndBindPorts 处理
bool Mid360DriverNode::tryInitDriver(uint16_t pclPort, uint16_t imuPort) {
    try {
        driver_ = std::make_unique<Mid360Driver>(
            ioContext_, hostIp_, pclPort, imuPort,
            [this](std::vector<Point> points) {
                onPointCloudReceived(std::move(points)); },
            [this](const ImuMessage& message) {
                onImuReceived(message); });
        return true;
    } catch (const std::exception& error) {
        return false;
    }
}

// ── attemptConnectAndStart ──
// 启动序列核心 (静默失败, 不输出 WARN):
//   discover (或直连) → writePushDest → 参数配置 → queryWorkState
// 雷达可能已在采样模式且保留了推送目标 (上一会话遗留),
// 此时 discovery / writePushDest 都可能失败, 但数据通道已就绪。
// 返回:
//   true  = 数据通道已建立 (discover 成功 或 queryWorkState 返回采样)
//   false = 雷达不可达, 由调用方决定重连策略
bool Mid360DriverNode::attemptConnectAndStart() {
    std::string sn, ip;
    uint16_t port = 0;

    // ── 发现或直连 ──
    if (targetLidarIp_ == "auto") {
        writeDebugLog("INFO", "DISCOVERY_START", "broadcasting discovery");
        bool discovered = driver_->discover(
            "255.255.255.255", sn, ip, port, 2000);
        if (discovered) {
            driver_->setRadarTarget(ip, port);
            RCLCPP_INFO(get_logger(), "发现雷达 SN=%s IP=%s port=%d",
                        sn.c_str(), ip.c_str(), port);
            writeDebugLog("INFO", "DISCOVERY_OK",
                std::string("SN=") + sn + " IP=" + ip + " port=" + std::to_string(port));
        } else {
            writeDebugLog("INFO", "DISCOVERY_FAIL", "no radar on broadcast");
        }
    } else {
        driver_->setRadarTarget(targetLidarIp_, kPortControlRadar);
        RCLCPP_INFO(get_logger(), "直连 %s", targetLidarIp_.c_str());
        writeDebugLog("INFO", "DIRECT_CONNECT", "target=" + targetLidarIp_);
    }

    // ── 配置推送目标 (拆为独立命令, 避免单条失败回滚影响其他) ──
    driver_->writePushDest(KEY_PCL_IPCFG,  pclPort_);
    driver_->writePushDest(KEY_IMU_IPCFG,  imuPort_);
    driver_->writePushDest(KEY_STATE_IPCFG, kHostPushBase);
    driver_->setPclDataType(pclFormat_);
    driver_->setImuEnabled(isImuEnabled_);
    driver_->setDetectMode(detectMode_);
    driver_->setWorkMode(WORK_MODE_SAMPLING);

    // ── 快速轮询工作状态 (最多 30 × 100ms = 3s) ──
    // 即使 discovery 失败, queryWorkState 也可能成功 (雷达已在运行)
    uint8_t state = 0;
    if (driver_->queryWorkState(state)) {
        if (state == STATE_SAMPLING) {
            RCLCPP_INFO(get_logger(), "雷达已在采样模式");
            writeDebugLog("INFO", "STATE_SAMPLING", "radar already sampling");
            return true;
        }
        RCLCPP_INFO(get_logger(), "等待电机启动 (%s)...", workStateName(state));
        for (int tick = 0; tick < 30; ++tick) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (driver_->queryWorkState(state) && state == STATE_SAMPLING) {
                RCLCPP_INFO(get_logger(), "雷达已进入采样");
                writeDebugLog("INFO", "STATE_SAMPLING",
                    "motor started after " + std::to_string((tick + 1) * 100) + "ms");
                return true;
            }
        }
        RCLCPP_INFO(get_logger(), "采样超时, 继续发布 (数据可能已到达)");
        writeDebugLog("INFO", "STATE_TIMEOUT",
                      std::string("state=") + workStateName(state));
        return true;  // 数据通道就绪
    }

    // 雷达不可达 — 返回 false, 由调用方触发后台重连
    return false;
}

// ── onSetSampling (服务回调) ──
// SetBool: data=true → SAMPLING, data=false → STANDBY
// 线程安全: null guard + cmdMutex_ 保护 driver_ 访问
void Mid360DriverNode::onSetSampling(
    std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

    if (!driver_) {
        response->success = false;
        response->message = "driver not initialized";
        return;
    }

    uint8_t mode = request->data ? WORK_MODE_SAMPLING : WORK_MODE_STANDBY;
    const char* modeName = request->data ? "sampling" : "standby";

    bool success;
    { std::lock_guard<std::mutex> lock(cmdMutex_);
      success = driver_->setWorkMode(mode); }

    response->success = success;
    response->message = success
        ? (std::string("switched to ") + modeName)
        : "radar rejected";

    writeDebugLog(success ? "INFO" : "WARN", "SVC_SET_SAMPLING",
        std::string("mode=") + modeName + " success=" + std::to_string(success));
}

// ── onGetWorkState (服务回调) ──
// Trigger: 返回当前工作状态名称
void Mid360DriverNode::onGetWorkState(
    std::shared_ptr<std_srvs::srv::Trigger::Request>  /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    if (!driver_) {
        response->success = false;
        response->message = "driver not initialized";
        return;
    }

    std::lock_guard<std::mutex> lock(cmdMutex_);
    uint8_t state;
    if (!driver_->queryWorkState(state)) {
        response->success = false;
        response->message = "query failed";
        return;
    }
    response->success = true;
    response->message = workStateName(state);
}

// ── onPointCloudReceived (io 线程 → 生产者) ──
// 接收解析后的点云 vector (move 语义, 零拷贝)
// 逻辑:
//   容量超限 (250K 点) → FIFO 截断最早的点 (而非全部清空, 保留最新数据)
//   pclBufferA_ 为空 → move 转移所有权 (O(1) 指针交换) + 恢复预留容量
//   pclBufferA_ 非空 → insert 追加 (使用 move_iterator 消除拷贝开销)
void Mid360DriverNode::onPointCloudReceived(std::vector<Point> points) {
    std::lock_guard<std::mutex> lock(pclMutex_);
    if (driver_ && driver_->isPtpSynced() && !ptpLogged_) {
        ptpLogged_ = true;
        RCLCPP_INFO(get_logger(), "PTP 时间同步已锁定");
        writeDebugLog("INFO", "PTP_SYNC_LOCKED", "time_type=1, PTP time sync locked");
    }
    size_t incoming = points.size();
    size_t current  = pclBufferA_.size();
    if (current + incoming > kMaxBufferPoints) [[unlikely]] {
        if (!pclGuardWarned_) {
            RCLCPP_WARN(get_logger(),
                "pclBufferA_ overflow (%zu limit), FIFO-truncating oldest points. "
                "Increase publish_freq_hz.", kMaxBufferPoints);
            pclGuardWarned_ = true;
        }
        // FIFO 截断: 保留最新数据, 丢弃缓冲区头部最早的点
        size_t excess = current + incoming - kMaxBufferPoints;
        if (excess < current) {
            pclBufferA_.erase(pclBufferA_.begin(), pclBufferA_.begin() + excess);
        } else {
            pclBufferA_.clear();
        }
    }
    if (pclBufferA_.empty()) {
        pclBufferA_ = std::move(points);
        pclBufferA_.reserve(kMaxBufferPoints);   // move 丢失旧容量 → 恢复
    } else {
        pclBufferA_.insert(pclBufferA_.end(),
            std::make_move_iterator(points.begin()),
            std::make_move_iterator(points.end()));
    }
}

// ── onImuReceived (io 线程 → 生产者) ──
// 单条 IMU 消息推入 imuBufferA_ (常引用, 拷贝)
void Mid360DriverNode::onImuReceived(const ImuMessage& message) {
    std::lock_guard<std::mutex> lock(imuMutex_);
    imuBufferA_.push_back(message);
}

// ── shouldKeepPoint (点云过滤, 发布时调用) ──
// 过滤流程: NaN/Inf 防御 → 距离² 检查 → X/Y/Z 轴范围检查
// 预计算的平方阈值避免 sqrt 调用
inline bool Mid360DriverNode::shouldKeepPoint(float x, float y, float z) const noexcept {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
    if (isDistFilterEnabled_) {
        float d2 = x * x + y * y + z * z;
        if (minDistSquared_ >= 0 && d2 < minDistSquared_) return false;
        if (maxDistSquared_ >= 0 && d2 > maxDistSquared_) return false;
    }
    if (isXFilterOn_ && (x < xMin_ || x > xMax_)) return false;
    if (isYFilterOn_ && (y < yMin_ || y > yMax_)) return false;
    if (isZFilterOn_ && (z < zMin_ || z > zMax_)) return false;
    return true;
}

// ── writePointToBuffer (发布辅助) ──
// 将单个 Point 结构体序列化为 25 字节 (layout 与 kPointFields 严格对齐)
// x,y,z,intensity → 4B float; timestamp → 8B double; tag → 1B uint8
inline void Mid360DriverNode::writePointToBuffer(uint8_t* dest, const Point& point) {
    std::memcpy(dest,      &point.x,          4);
    std::memcpy(dest + 4,  &point.y,          4);
    std::memcpy(dest + 8,  &point.z,          4);
    std::memcpy(dest + 12, &point.intensity,  4);
    std::memcpy(dest + 16, &point.timestamp,  8);
    dest[24] = point.tag;
}

// ── publishPointCloud (定时发布, timer 线程) ──
// 双缓冲消费:
//   swap(pclBufferA_, pclBufferB_) → 清空 A → 释放锁
//   → 序列化 B 到 pclOutputBuffer_ (可选过滤)
//   → 构造 PointCloud2 → swap data 段 (零拷贝) → publish
// 性能: 10Hz × 20K 点 = 200K pts/s → ~500KB memcpy + 20K 过滤检查
void Mid360DriverNode::publishPointCloud() {
    { std::lock_guard<std::mutex> lock(pclMutex_);
      std::swap(pclBufferA_, pclBufferB_);
      pclBufferA_.clear(); }
    if (pclBufferB_.empty()) return;

    constexpr size_t POINT_SIZE = 25;
    size_t totalPoints = pclBufferB_.size();
    pclOutputBuffer_.resize(totalPoints * POINT_SIZE);
    size_t  keptCount = 0;
    uint8_t* outputPtr = pclOutputBuffer_.data();

    // 无过滤时直接序列化; 有过滤时每点先检查再序列化
    if (isFilterDisabled_) {
        for (size_t i = 0; i < totalPoints; ++i) {
            writePointToBuffer(outputPtr + keptCount * POINT_SIZE,
                               pclBufferB_[i]);
            ++keptCount;
        }
    } else {
        for (size_t i = 0; i < totalPoints; ++i) {
            const auto& point = pclBufferB_[i];
            if (!shouldKeepPoint(point.x, point.y, point.z)) continue;
            writePointToBuffer(outputPtr + keptCount * POINT_SIZE, point);
            ++keptCount;
        }
    }
    if (!keptCount) return;

    // 构造 ROS2 PointCloud2 消息
    sensor_msgs::msg::PointCloud2 message;
    double firstTimestamp = pclBufferB_[0].timestamp;
    // 时间戳恒为正, static_cast 等价于 std::floor (省去函数调用)
    message.header.stamp.sec     = static_cast<int32_t>(firstTimestamp);
    message.header.stamp.nanosec = static_cast<uint32_t>(
        (firstTimestamp - message.header.stamp.sec) * 1e9);
    message.header.frame_id = pclFrameId_;
    message.fields       = kPointFields;
    message.is_bigendian = false;
    message.point_step   = POINT_SIZE;
    message.is_dense     = true;
    message.height       = 1;
    message.width        = keptCount;
    message.row_step     = keptCount * POINT_SIZE;

    // 零拷贝移交 data 段所有权 → re-reserve 供下一周期使用
    pclOutputBuffer_.resize(keptCount * POINT_SIZE);
    message.data.swap(pclOutputBuffer_);
    pclOutputBuffer_.reserve(totalPoints * POINT_SIZE);

    pclPublisher_->publish(message);
}

// ── publishImuMessages (定时发布, timer 线程) ──
// 与点云相同的双缓冲模式。
// IMU 固定 1ms 轮询, 200Hz 数据量 (每周期平均 0.2 条消息)
// REP-145: 未提供协方差 → 置 -1 (unknown)
void Mid360DriverNode::publishImuMessages() {
    { std::lock_guard<std::mutex> lock(imuMutex_);
      std::swap(imuBufferA_, imuBufferB_);
      imuBufferA_.clear(); }

    for (auto& imu : imuBufferB_) {
        sensor_msgs::msg::Imu message;
        message.header.frame_id = imuFrameId_;
        message.header.stamp.sec     = static_cast<int32_t>(imu.timestamp);
        message.header.stamp.nanosec = static_cast<uint32_t>(
            (imu.timestamp - message.header.stamp.sec) * 1e9);
        message.angular_velocity.x     = imu.angularVelocityX;
        message.angular_velocity.y     = imu.angularVelocityY;
        message.angular_velocity.z     = imu.angularVelocityZ;
        message.linear_acceleration.x  = imu.linearAccelX;
        message.linear_acceleration.y  = imu.linearAccelY;
        message.linear_acceleration.z  = imu.linearAccelZ;
        // 协方差: 未知 = -1 (REP-145)
        message.orientation_covariance[0]         = -1.0;
        message.angular_velocity_covariance[0]    = -1.0;
        message.linear_acceleration_covariance[0] = -1.0;
        imuPublisher_->publish(message);
    }
}

// ══════════════════════════════════════════════════════════════════════════════
//  Debug 日志系统
// ══════════════════════════════════════════════════════════════════════════════
//
//  格式: JSON lines (适配 MissionLogger 风格, 方便 agent 解析)
//  {"t":"HH:MM:SS.mmm","lvl":"INFO","mod":"mid360_driver","evt":"EVENT","msg":"..."}
//
//  文件路径: {log_base_path}/{YYYYMMDD_HHMMSS}/mid360_driver.trc
//  若 log_base_path 不存在: 输出 RCLCPP_WARN 并创建目录后继续写入

// ── initDebugLog ──
// 在 loadParameters 之后调用。
// 创建日志基目录 (如不存在, 告警后创建) → 创建时间戳子目录 → 打开 .trc 文件
void Mid360DriverNode::initDebugLog() {
    if (!debugLogEnabled_) return;

    // 确保路径以 / 结尾
    if (!logBasePath_.empty() && logBasePath_.back() != '/')
        logBasePath_ += '/';

    // 基目录不存在 → 告警 + 创建
    if (!std::filesystem::exists(logBasePath_)) {
        RCLCPP_WARN(get_logger(), "log base dir not found, creating: %s",
                    logBasePath_.c_str());
        std::filesystem::create_directories(logBasePath_);
    }

    // 时间戳子目录
    auto now = std::chrono::system_clock::now();
    auto timeT = std::chrono::system_clock::to_time_t(now);
    std::tm localTm;
    localtime_r(&timeT, &localTm);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &localTm);
    logStampDir_ = logBasePath_ + stamp + "/";
    std::filesystem::create_directories(logStampDir_);

    // 打开 trace 文件
    std::string logPath = logStampDir_ + "mid360_driver.trc";
    debugLogFile_.open(logPath, std::ios::out | std::ios::trunc);
    if (debugLogFile_.is_open()) {
        writeDebugLog("INFO", "LOG_OPEN", "debug log started");
        RCLCPP_INFO(get_logger(), "debug log: %s", logPath.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "failed to open debug log: %s", logPath.c_str());
    }
}

// ── writeDebugLog ──
// 写入一条 JSON line 格式的日志。
// 时间精度: HH:MM:SS.mmm (毫秒)
// 线程安全: 内部 debugLogMutex_ 保护文件写入
void Mid360DriverNode::writeDebugLog(const std::string& level,
                                     const std::string& event,
                                     const std::string& msg) {
    if (!debugLogEnabled_ || !debugLogFile_.is_open()) return;

    std::lock_guard<std::mutex> lock(debugLogMutex_);

    // 构造时间戳 HH:MM:SS.mmm
    auto now = std::chrono::system_clock::now();
    auto timeT = std::chrono::system_clock::to_time_t(now);
    std::tm localTm;
    localtime_r(&timeT, &localTm);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    char timeBuf[16];
    std::strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", &localTm);

    // JSON line: {"t":"HH:MM:SS.mmm","lvl":"INFO","mod":"mid360_driver","evt":"EVT","msg":"..."}
    debugLogFile_ << "{\"t\":\"" << timeBuf << "."
                  << std::setfill('0') << std::setw(3) << ms.count()
                  << "\",\"lvl\":\"" << level
                  << "\",\"mod\":\"mid360_driver\""
                  << ",\"evt\":\"" << event
                  << "\",\"msg\":\"" << msg << "\"}" << std::endl;
}

}  // namespace mid360_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mid360_driver::Mid360DriverNode)
