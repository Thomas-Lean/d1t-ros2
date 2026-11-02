# 打印检查说明

状态：**NOT FOR REAL CALIBRATION UNTIL MEASURED**。

1. 打开 `VISION-01-charuco-design-not-measured.pdf`，仅打印第 1 页；选择 A4、100% / Actual size，关闭“适应纸张”“缩放”“无边距自动缩放”。
2. 用卡尺测量棋盘黑白外边界的横向和纵向，二者都应为 150.0 mm；再测多处方格边长应为 30.0 mm、marker 边长应为 22.0 mm。记录测量工具和结果。
3. 横纵打印比例不一致、板面弯曲、边界被裁切、污渍遮挡 marker，均判定失败并重新制作；不能靠软件改 `print_scale` 掩盖。
4. 合格实物仍需在真实 runtime 配置中填入实测米制尺寸、实际/名义打印比例、物理 board ID、测量记录，并将 `measured` 设为 true。仅修改 JSON 的 `measured` 不构成验证。

本目录保存的是离线设计和校验，不采集相机数据，也不构成 VISION-01 的真实标定结果。

## 2026-09-20 已打印件结果

本次实测 X=149.5 mm、Y=151.0 mm，横纵比例不一致，判定失败；不得使用该打印件做真实标定。详见 `PRINT-MEASUREMENT-20260920.md`。
