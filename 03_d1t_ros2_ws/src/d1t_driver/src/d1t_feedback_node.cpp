// d1t_feedback_node.cpp
//
// Unitree D1-T 机械臂的反馈解析节点：只读取反馈，不向机械臂发送任何命令。
//
// 数据流：
//   D1-T 控制器 --(DDS)--> /arm_Feedback (消息类型：unitree_arm/msg/ArmString)
//   ArmString::data 是 JSON 格式的字符串，例如：
//       {"seq":10,"address":2,"funcode":1,"data":{"angle0":0.0,...,"angle6":49.8}}
//       {"seq":10,"address":2,"funcode":3,"data":{"enable_status":0,"power_status":0,"error_status":0}}
//       {"seq":N ,"address":3,"funcode":1,"data":{"recv_status":...}}   命令接收回执
//       {"seq":N ,"address":3,"funcode":2,"data":{"exec_status":...}}   命令执行状态回执
//
// 本节点订阅 /arm_Feedback，并把解析结果发布到三个派生话题（/d1t/joint_feedback、
// /d1t/arm_status、/d1t/command_ack）；不修改 /arm_Feedback，不创建 /arm_Command 发布者，
// 也不执行使能、上电或运动。

#include <array>
#include <cstdint>
#include <exception>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unitree_arm/msg/arm_string.hpp"

#include "d1t_interfaces/msg/joint_feedback.hpp"
#include "d1t_interfaces/msg/arm_status.hpp"
#include "d1t_interfaces/msg/command_ack.hpp"

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using unitree_arm::msg::ArmString;
using d1t_interfaces::msg::JointFeedback;
using d1t_interfaces::msg::ArmStatus;
using d1t_interfaces::msg::CommandAck;

namespace
{
// 关节角度和机械臂状态约以 10 Hz 发布。日志按此时间间隔限流，避免终端被重复反馈刷屏。
constexpr int64_t kThrottleMs = 2000;

// 从 JSON 对象读取一个整数值字段。
// 字段不存在或值不是数值时返回 false；成功时把转换后的值写入 out。
bool get_int(const json & obj, const char * key, int64_t & out)
{
  auto it = obj.find(key);
  if (it == obj.end() || !it->is_number()) {
    return false;
  }
  out = it->get<int64_t>();
  return true;
}

// 从 JSON 对象读取一个浮点值字段。失败条件与 get_int 相同。
bool get_double(const json & obj, const char * key, double & out)
{
  auto it = obj.find(key);
  if (it == obj.end() || !it->is_number()) {
    return false;
  }
  out = it->get<double>();
  return true;
}

// 把可能为 -1（缺失）或超出 uint32 范围的 seq 收敛为 uint32。
// 仅用于关节/状态帧：这两类帧不因 seq 缺失而丢弃，缺失时以 0 填充。
uint32_t seq_to_u32(int64_t seq)
{
  if (seq < 0 || seq > static_cast<int64_t>(UINT32_MAX)) {
    return 0u;
  }
  return static_cast<uint32_t>(seq);
}
}  // namespace

class D1TFeedbackNode : public rclcpp::Node
{
public:
  D1TFeedbackNode()
  : Node("d1t_feedback_node")
  {
    // QoS 必须与 DDS 发布端兼容：可靠传输、易失性（不保存历史消息）。
    // 深度取 10：同一刻关节帧与状态帧相邻到达时，KeepLast(1) 会互相顶掉，留 10 条缓冲。
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability_volatile();

    // 保存订阅对象的成员变量 sub_，防止构造函数结束后订阅对象被销毁。
    // 回调中的 _1 是 DDS 收到的 ArmString 消息。
    sub_ = create_subscription<ArmString>(
      "/arm_Feedback", qos,
      std::bind(&D1TFeedbackNode::on_feedback, this, std::placeholders::_1));

    // 派生话题的发布 QoS：可靠 + 易失；关节/状态各保留最近 10 条，命令回执保留最近 20 条。
    rclcpp::QoS derived_qos(rclcpp::KeepLast(10));
    derived_qos.reliable();
    derived_qos.durability_volatile();
    rclcpp::QoS ack_qos(rclcpp::KeepLast(20));
    ack_qos.reliable();
    ack_qos.durability_volatile();

    joint_pub_ = create_publisher<JointFeedback>("/d1t/joint_feedback", derived_qos);
    arm_status_pub_ = create_publisher<ArmStatus>("/d1t/arm_status", derived_qos);
    command_ack_pub_ = create_publisher<CommandAck>("/d1t/command_ack", ack_qos);

    RCLCPP_INFO(
      get_logger(),
      "d1t_feedback_node started: subscribing /arm_Feedback; publishing "
      "/d1t/joint_feedback, /d1t/arm_status, /d1t/command_ack "
      "(no /arm_Command publisher created)");
  }

private:
  void on_feedback(const ArmString::SharedPtr msg)
  {
    // DDS 消息的载荷是文本，先反序列化为 JSON；格式错误的报文只记录错误，不影响节点继续接收。
    json root;
    try {
      root = json::parse(msg->data);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), kThrottleMs,
        "arm_Feedback JSON parse error: %s | raw=%s", e.what(), msg->data.c_str());
      return;
    }

    // 仅接受 JSON 对象，例如 {"address":2, "funcode":1, ...}；数组、字符串等均视为非法报文。
    if (!root.is_object()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), kThrottleMs,
        "arm_Feedback JSON is not an object: %s", msg->data.c_str());
      return;
    }

    // address 标识报文对象，funcode 标识该对象下的报文类别；二者共同决定分发到哪个处理函数。
    int64_t address = 0;
    int64_t funcode = 0;
    if (!get_int(root, "address", address) || !get_int(root, "funcode", funcode)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), kThrottleMs,
        "arm_Feedback missing/invalid address or funcode: %s", msg->data.c_str());
      return;
    }

    // seq 是控制器的序号，仅用于关联和诊断；当前节点不依赖它，因此缺失不丢弃报文。
    int64_t seq = -1;
    get_int(root, "seq", seq);

    // 正常协议把具体字段放在 data 对象内。若控制器发送扁平结构，则退回到根对象解析，
    // 从而兼容 {"address":2, "funcode":1, "angle0":...} 这类报文。
    auto data_it = root.find("data");
    const json & payload =
      (data_it != root.end() && data_it->is_object()) ? *data_it : root;

    if (address == 2 && funcode == 1) {
      handle_joints(seq, payload, msg->data);
    } else if (address == 2 && funcode == 3) {
      handle_arm_status(seq, payload, msg->data);
    } else if (address == 3 && funcode == 1) {
      handle_recv_status(payload, root, msg->data);
    } else if (address == 3 && funcode == 2) {
      handle_exec_status(payload, root, msg->data);
    } else {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), kThrottleMs,
        "unhandled arm_Feedback address=%ld funcode=%ld", address, funcode);
    }
  }

  // address=2、funcode=1：7 个关节角度，单位为度；属于高频反馈，日志需要限流。
  void handle_joints(int64_t seq, const json & payload, const std::string & raw)
  {
    // 固定按 j0 至 j6 的顺序保存，零初始化可避免后续误用未写入的数据。
    std::array<double, 7> angles{};
    for (int i = 0; i < 7; ++i) {
      // JSON 键名由循环生成：angle0、angle1 …… angle6。
      const std::string key = "angle" + std::to_string(i);
      if (!get_double(payload, key.c_str(), angles[static_cast<size_t>(i)])) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), kThrottleMs,
          "joint feedback missing/invalid '%s': %s", key.c_str(), raw.c_str());
        return;
      }
    }
    // 只有 7 个字段全部有效，才计入一条完整关节反馈。
    ++joint_msg_count_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), kThrottleMs,
      "joints deg [j0..j6] = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] (#%lu)",
      angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6],
      static_cast<unsigned long>(joint_msg_count_));

    // 7 轴齐全后，除保留原日志外，再发布一条结构化关节反馈。
    // angle6 未标定，按原样放入 angle6_raw，不做单位换算。
    JointFeedback jf;
    jf.seq = seq_to_u32(seq);
    jf.angle0 = angles[0];
    jf.angle1 = angles[1];
    jf.angle2 = angles[2];
    jf.angle3 = angles[3];
    jf.angle4 = angles[4];
    jf.angle5 = angles[5];
    jf.angle6_raw = angles[6];
    joint_pub_->publish(jf);
  }

  // address=2、funcode=3：使能、上电和错误状态；同样是高频反馈，日志需要限流。
  void handle_arm_status(int64_t seq, const json & payload, const std::string & raw)
  {
    int64_t enable_status = 0;
    int64_t power_status = 0;
    int64_t error_status = 0;
    if (!get_int(payload, "enable_status", enable_status) ||
      !get_int(payload, "power_status", power_status) ||
      !get_int(payload, "error_status", error_status))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), kThrottleMs,
        "arm status feedback missing/invalid enable/power/error field: %s", raw.c_str());
      return;
    }
    // 三种状态均有效后才输出状态，并累计完整状态报文数。
    ++status_msg_count_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), kThrottleMs,
      "arm status: enable_status=%ld power_status=%ld error_status=%ld (#%lu)",
      enable_status, power_status, error_status,
      static_cast<unsigned long>(status_msg_count_));

    // 三个状态齐全后，除保留原日志外，再发布一条结构化状态。只转发原始数值，不重新解释。
    ArmStatus st;
    st.seq = seq_to_u32(seq);
    st.enable_status = static_cast<int32_t>(enable_status);
    st.power_status = static_cast<int32_t>(power_status);
    st.error_status = static_cast<int32_t>(error_status);
    arm_status_pub_->publish(st);
  }

  // address=3、funcode=1：命令接收回执。此类事件由命令触发，逐条输出日志。
  void handle_recv_status(const json & payload, const json & root, const std::string & raw)
  {
    int64_t recv_status = 0;
    int64_t seq = -1;
    // 优先从 data 读取；兼容 recv_status 直接位于根对象的扁平报文。
    if (!get_int(payload, "recv_status", recv_status) &&
      !get_int(root, "recv_status", recv_status))
    {
      RCLCPP_ERROR(
        get_logger(), "recv_status feedback missing/invalid 'recv_status': %s", raw.c_str());
      return;
    }
    // seq 可能位于 data 或根对象；读取失败时保留 -1，日志仍能显示回执状态。
    if (!get_int(payload, "seq", seq)) {
      get_int(root, "seq", seq);
    }
    RCLCPP_INFO(get_logger(), "command recv echo: seq=%ld recv_status=%ld", seq, recv_status);

    // 命令回执必须携带原始 seq。缺失或超出 uint32 范围时记录错误并跳过发布，节点继续运行。
    if (seq < 0 || seq > static_cast<int64_t>(UINT32_MAX)) {
      RCLCPP_ERROR(
        get_logger(), "command_ack(RECV) not published, missing/invalid seq: %s", raw.c_str());
      return;
    }
    CommandAck ack;
    ack.seq = static_cast<uint32_t>(seq);
    ack.ack_type = CommandAck::RECV;
    ack.status = static_cast<int32_t>(recv_status);
    command_ack_pub_->publish(ack);
  }

  // address=3、funcode=2：命令执行状态回执。此类事件由命令触发，逐条输出日志。
  void handle_exec_status(const json & payload, const json & root, const std::string & raw)
  {
    int64_t exec_status = 0;
    int64_t seq = -1;
    // 与接收回执一致：优先解析 data，兼容扁平报文。
    if (!get_int(payload, "exec_status", exec_status) &&
      !get_int(root, "exec_status", exec_status))
    {
      RCLCPP_ERROR(
        get_logger(), "exec_status feedback missing/invalid 'exec_status': %s", raw.c_str());
      return;
    }
    // seq 缺失不会阻塞执行状态的显示。
    if (!get_int(payload, "seq", seq)) {
      get_int(root, "seq", seq);
    }
    RCLCPP_INFO(get_logger(), "command exec echo: seq=%ld exec_status=%ld", seq, exec_status);

    // 命令回执必须携带原始 seq。缺失或超出 uint32 范围时记录错误并跳过发布，节点继续运行。
    if (seq < 0 || seq > static_cast<int64_t>(UINT32_MAX)) {
      RCLCPP_ERROR(
        get_logger(), "command_ack(EXEC) not published, missing/invalid seq: %s", raw.c_str());
      return;
    }
    CommandAck ack;
    ack.seq = static_cast<uint32_t>(seq);
    ack.ack_type = CommandAck::EXEC;
    ack.status = static_cast<int32_t>(exec_status);
    command_ack_pub_->publish(ack);
  }

  // 持有订阅对象的生命周期；节点存活期间始终订阅 /arm_Feedback。
  rclcpp::Subscription<ArmString>::SharedPtr sub_;
  // 三个派生话题的发布器：结构化关节反馈、结构化状态、命令回执。
  rclcpp::Publisher<JointFeedback>::SharedPtr joint_pub_;
  rclcpp::Publisher<ArmStatus>::SharedPtr arm_status_pub_;
  rclcpp::Publisher<CommandAck>::SharedPtr command_ack_pub_;
  // 分别统计通过完整字段校验的关节和状态反馈数量，仅用于日志诊断。
  uint64_t joint_msg_count_ = 0;
  uint64_t status_msg_count_ = 0;
};

int main(int argc, char ** argv)
{
  // 初始化 ROS 2 通信上下文，创建节点并进入阻塞式事件循环。
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<D1TFeedbackNode>());
  // spin 返回后释放 ROS 2 的全局资源，再以成功状态退出进程。
  rclcpp::shutdown();
  return 0;
}
