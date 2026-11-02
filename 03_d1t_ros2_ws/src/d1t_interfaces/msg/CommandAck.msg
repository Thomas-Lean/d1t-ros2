# 由 d1t_feedback_node 从 /arm_Feedback (address=3, funcode=1 或 2) 派生。
# seq 必须使用反馈中的原始值；缺失或类型错误时不发布。

uint8 RECV=1
uint8 EXEC=2

uint32 seq
uint8 ack_type
int32 status
