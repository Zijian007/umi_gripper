# 使用教程

安装包及维护命令见 [安装教程](install.md)。标定、改 ID 和试跑都针对实际连接的夹爪。

## 标定与改 ID

从项目目录运行标定。配置路径默认为当前目录的 `gripper_config.json`，也可显式指定。

```bash
umi-calibrate --config ./gripper_config.json
```

按提示扫描串口并选择通道。空通道选 `0`。工具在设备会话里设置单圈绝对位置模式和舵机中位，随后依次标定闭合、中立、张开。用 W/S 步进，回车确认，Q 取消。完成三点后可执行夹紧力度测试。每完成一个通道，工具将已完成的通道保存到 JSON。加 `--log` 会周期显示位置和反馈。

核对或修改舵机 ID 使用 `umi-id`。改 ID 时同一串口上只接目标舵机。改完后将新 ID 更新到标定配置，或重新标定。

## 交互试跑

```bash
umi-demo --config ./gripper_config.json
```

例如输入 `0.8 0`，让 0 号通道移动至 80% 开度。输入 `q` 结束会话。

## 应用 API

`UmiGripper.set_target(close_01)` 接收闭合比例，`0` 为全开，`1` 为全闭。调用提交目标后返回。应用可读取 `UmiReading` 的状态和采样时间判断到位或故障。

```python
from umi_gripper import UmiGripper

gripper = UmiGripper.open(
    side="right",
    options={"channel": 1, "config_path": "./gripper_config.json"},
)
try:
    gripper.set_target(0.2)
    print(gripper.read())
finally:
    gripper.close()
```

`read()` 返回缓存的最后一次反馈，重复读取沿用其单调时钟采样时间。读数可出现 `arrived`、`grasped`、`blocked` 或 `error`。关闭最后一个通道会等待轮询线程退出再释放串口。若关闭报告超时或串口异常，保留原对象并重试 `close()`。若打开失败且抛出的 `UmiError.resource` 带有对象，通过该对象重试 `close()`。

旧 `from gripper import Gripper` 入口仍支持 `goto(width)`。它的 `1` 表示全开，并提供 `wait()` 和上下文管理器。离线测试可传 `bus_factory=FakeServoBus`，该类位于 `umi_gripper.fake_bus`。标定模式下可使用手动步进入口，直到提交有效三点。

## 接入 wholebody

在 wholebody 的 `config_data/hardware/robot.yaml` 选择夹爪类型。

```yaml
hardware:
  grippers:
    left:
      backend: omnipicker
    right:
      backend: umi
```

UMI 侧从 `config_data/hardware/grippers/umi.yaml` 读取运行参数。YAML 可覆盖同名速度、力度、反馈频率和保护阈值。标定 JSON 保存串口、舵机 ID、波特率和行程。默认标定文件是相对 wholebody 根目录的 `../umi_gripper/gripper_config.json`。同一配置文件的通道共用会话。一侧为空时，wholebody 可映射到另一个已标定通道。

## 配置字段

JSON 有两个标定槽，`servos[0]` 对应左侧，`servos[1]` 对应右侧。空槽填 `null`。

| 字段 | 用途 |
| --- | --- |
| `serial_port`、`servo_id`（0–253）、`baudrate`（9600–1000000） | 读，用来开串口 |
| `closed_position_steps`、`open_position_steps`（0–4095） | 读。`close_01` 0 → 张开步数，1 → 闭合步数 |
| `neutral_position_steps` | 校验行程并写入录制元数据。旧接口的 `reset()` 将它作为目标位置 |
| `speed`（1–3400）、`open_grip_strength` / `close_grip_strength`（1–100）、`status_frequency_hz`（1–100） | 先读 JSON，YAML 里有同名项就覆盖。`speed` 原样写入地址 46，见运行时写入表 |
| `accel`（0–254）、`position_tolerance_steps`（0–128）、`stall_movement_steps`（1–128）、`opening_stall_window_s` / `closing_stall_window_s`（0.05–5 秒）、`overtemp_c`（1–100）、`overtemp_clear_c`（0–99，且低于触发温度）、`overtemp_hold_s`（0–5 秒） | YAML 可覆盖 JSON。缺省值为加速度 50、到位 12 步、堵住 3 步、张开堵转窗 0.5 秒、闭合堵转窗 0.2 秒、过温 70℃ 持续 0.5 秒、65℃ 解除。`accel` 为 0 时使用舵机最大加速度 |
| `servo_model_number`、`calibration_step_steps`、`motion_timeout_s` | 分别用于标定记录、手动步进和兼容接口的等待超时 |

张开和闭合力度分别由 `open_grip_strength` 和 `close_grip_strength` 提供。旧标定可以用原有 `Gripper` 入口加载后调用 `save_config`，新文件记录两个独立字段。

## 运行时写入

下表列出 wholebody 运行时写入的飞特 STS SRAM 地址。上电时，舵机将 EEPROM 地址 16 的最大扭矩复制到地址 48。常见出厂值为 1000，即堵转扭矩的 100%。

| 地址 | 何时写 | 值 |
| --- | --- | --- |
| 40 扭矩开关 | 打开通道时写一次 | `1`（使能）。关闭会话后舵机仍处于使能状态 |
| 48 转矩限制，2 字节，单位 0.1% | 打开时按闭合力度写一次；之后只有力度变化才再写 | `力度 × 10`。40 → 400，即堵转扭矩的 40%。张开用 `open_grip_strength`，闭合用 `close_grip_strength` |
| 41 加速度 | 每次 `WritePosEx` | 配置里的 `accel`。单位 8.7°/s²。写 0 表示最大加速度 |
| 42–43 目标位置 | 下发、到位、停止 | 写入目标步数。到位或停止时写入当前步数 |
| 44–45 PWM 开环速度 | 每次 `WritePosEx` | 固定为 0。此字段仅在运行模式 2 生效 |
| 46–47 运行速度 | 每次 `WritePosEx` | 配置里的 `speed`。相位 bit2 为 0 时单位是 0.732 RPM。wholebody 写入的数值使方向位 bit15 为 0。配置范围为 1–3400，实际速度受舵机上限约束 |

到达 `position_tolerance_steps` 范围内时，驱动将目标设为当前位置。闭合时，若在 `closing_stall_window_s` 内移动不足 `stall_movement_steps`，状态为 `grasped`，舵机继续以闭合转矩限制保持原目标。张开时，同样的移动阈值配合 `opening_stall_window_s` 判定 `blocked`，驱动将目标设为当前位置。温度达到 `overtemp_c` 并持续 `overtemp_hold_s` 后，驱动停在当前位置并报告 `error`。降至 `overtemp_clear_c` 或更低后可接收新目标。

## 运行时反馈

状态频率由 `status_frequency_hz` 决定。每个周期读取一次地址 56–70，从中取位置、占空比、温度和电流。设备返回 `UmiReading`，由 wholebody 转换为 `GripperReading`。`close_01` 由当前位置在张开和闭合步数之间换算。`raw_position` 的单位为步，录制字段为 `position_raw`，配套字段为 `position_unit: steps`。

SDK 超时返回 `-6`（no status packet）。驱动取消当前目标并报告错误。下一次成功读取清除通信错误，后续动作由应用重新下发。

| 地址 | 含义 | 怎么用 |
| --- | --- | --- |
| 33 模式 | 打开通道时读取一次 | 运行要求为 0（绝对位置） |
| 56–57 当前位置 | 2 字节，0–4095 | `close_01`、`raw_position`（录制为 `position_raw`）。超出标定行程（容差 `position_tolerance_steps`，和到位共用）视为无效 |
| 60–61 当前负载 | bit10 是方向，幅值 0–1000，单位 0.1% 驱动电压占空比 | `effort` 保存带符号的原始值。调试脚本显示 `effort / 10`，即带符号的占空比百分比 |
| 69–70 当前电流 | bit15 是方向，单位 6.5 mA | `current` 为安培：先把原始值乘 6.5 后四舍五入成毫安，再除以 1000 |
| 63 当前温度 | 1 字节，℃ | `temperature`。达到 `overtemp_c` 并持续 `overtemp_hold_s` 时停止。降至 `overtemp_clear_c` 或更低时清除过温错误 |
