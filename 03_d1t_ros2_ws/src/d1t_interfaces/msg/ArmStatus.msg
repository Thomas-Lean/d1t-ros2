# 由 d1t_feedback_node 从 /arm_Feedback (address=2, funcode=3) 派生。
# 只转发厂商原始数值，不重新解释；不得根据 power_status 推断使能状态。

uint32 seq
int32 enable_status
int32 power_status
int32 error_status
