# 紧凑双齿条升降夹爪（审阅版）

本版本在 Z 齿条下方增加小型夹爪安装平台。平台贴在齿条前侧，避免进入升降小齿轮的齿宽；夹爪槽口朝下，夹爪用 2×M3 螺钉安装，孔中心距 24 mm。Z 轴名义行程为 80 mm，使用被固定通道约束的滑块导向，不使用 Z 轴光轴。

## 文件

- `outputs/assembly_open_retracted_REVIEW_ONLY.stl`：张开、低位总装审阅网格。
- `outputs/assembly_open_extended_REVIEW_ONLY.stl`：张开、高位总装审阅网格。
- `outputs/z_claw_platform_2xM3.stl`：夹爪平台，约 66×14×8 mm，夹爪孔为 Ø3.4 mm、中心距 24 mm。
- `outputs/z_rack.stl`、`outputs/z_lift_carriage.stl`、`outputs/z_lift_frame.stl`：升降件。
- `outputs/validation.json`：尺寸、姿态和网格交叠检查结果。

STL 是审阅用网格，不是原生 SolidWorks 特征模型或 STEP。实物舵机、轴承和紧固件仍需按型号核对。

## 当前关键参数

| 项目 | 参数 |
|---|---:|
| Z 行程 | 80 mm |
| Z 小齿轮 | 30 齿，模数 2，节圆半径 30 mm |
| Z 齿条 | 约 105 mm 长 |
| 夹爪同步齿轮 | 12 齿，模数 1.5 |
| 夹爪张开基准间距 | 90 mm |
| 高度范围 | 低位总高 272 mm；最高基准 280 mm |
| 夹爪平台 | 约 66×14×8 mm；2×M3 安装孔，24 mm 中心距 |

## 校验状态

构建可完成，输出网格是单壳体打印件，升降姿态按 5 mm 步距采样，最大名义行程 80 mm。`validation.json` 仍列有总装内部交叠，包括导向通道与夹爪骨架区域；因此本版用于继续审阅，不能当作无干涉的制造定版。夹持力、舵机扭矩、打印公差和结构强度均未做实物验证。夹持 Ø35 mm 参考球的几何接触也尚未验证。
