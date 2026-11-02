# 由 d1t_feedback_node 从 /arm_Feedback (address=2, funcode=1) 派生。
# 只在 7 个关节字段全部存在且均为数值时发布。

uint32 seq

# angle0～angle5 是厂商反馈原始值，单位为度。
float64 angle0
float64 angle1
float64 angle2
float64 angle3
float64 angle4
float64 angle5

# angle6 保留厂商反馈原始值，量纲和原点尚未验证，不做单位转换。
float64 angle6_raw
