# 使用教程

标定、改 ID 和试跑都针对实际连接的夹爪。如果当前 Python 环境还未安装 UMI，先完成安装。[安装教程](install.md)

## 准备终端

激活已安装 `umi_gripper` 的 Python 环境。独立使用时，在 `umi_gripper` 仓库激活虚拟环境。

```bash
cd "$HOME/umi_gripper"
. .venv/bin/activate
```

标定、试跑和 Python API 共用默认标定文件。源码安装时使用 UMI 仓库根目录的 `gripper_config.json`，通过 wheel 安装时使用 `~/umi_gripper/gripper_config.json`。可以在任意工作目录运行以下命令，确认当前环境能找到工具和 Python 包。

```bash
command -v umi-id umi-calibrate umi-demo
python -c 'from umi_gripper import Gripper; print(Gripper.__module__)'
```

## 标定与改 ID

### 扫描和核对舵机

接好驱动板，在 Linux 上查看 USB 串口和稳定设备链接。

```bash
lsusb
python -m serial.tools.list_ports -v
ls -l /dev/serial/by-id/
```

确认舵机已供电、总线接线正确，通信板跳线位于 B 位置。运行舵机 ID 工具。

```bash
umi-id
```

1. 在扫描提示按 `Enter`，记录列表里的实际串口、舵机 ID 和波特率。
2. 输入列表序号并按 `Enter` 查看详细信息。在「输入新 ID」处直接按 `Enter`，完成核对并返回扫描提示。
3. 输入 `q` 并按 `Enter` 退出。

`umi-id` 逐个串口尝试波特率，找到有舵机回应的波特率后列出该速率下的所有 ID。`umi-id` 和 `umi-calibrate` 共用扫描流程，都会跳过打不开的串口。驱动板已经出现在串口列表里，但工具显示「未找到舵机」时，先检查设备属组和当前会话权限。将 `/dev/ttyACM0` 换成实际设备名。[串口权限检查](install.md#确认当前终端有串口权限)

```bash
ls -l /dev/ttyACM0
getent group dialout
id -nG
```

如果设备属组是 `dialout`，账号出现在 `getent` 的结果里，但 `id -nG` 没有 `dialout`，可在已激活的环境里用下面的命令临时扫描。重新登录后，用 `id -nG` 确认组权限已生效，激活环境即可直接运行维护命令。[准备终端](#准备终端)

```bash
sg dialout -c 'umi-id'
```

确认当前终端可以访问串口后，若仍无回应，再检查舵机供电、B 位跳线和总线接线。

双臂接入前，按实物确认每个 ID 对应哪一侧。如果接线无法确认，可分别只接一侧扫描来识别。共用串口的两只舵机使用相同波特率，并设置不同 ID。

### 修改舵机 ID（可选）

同一串口只接要改 ID 的那只舵机，再运行工具。

```bash
umi-id
```

按 `Enter` 扫描，输入目标舵机的列表序号，在「输入新 ID」处填写 `0` 到 `253` 内的计划值并按 `Enter`。工具会核对新 ID 的通信。看到成功提示后可再按 `Enter` 扫描核对，输入 `q` 退出。如果只修改了同一只舵机的 ID，可以将新 ID 更新到对应标定槽的 `servo_id`。

### 标定

首次接入、换夹爪或换舵机时，按实物确认左右对应后，重新标定受影响的通道。运行时使用当前设备生成的 `gripper_config.json`。[历史标定记录](worklog/records/gripper_config.historical.json)

`umi-calibrate` 默认写入 UMI 仓库根目录的 `gripper_config.json`，也可用 `--config` 指定另一个文件。每完成一侧就更新该槽，并原子替换配置文件。已有的另一侧标定会保留，两侧可以分开标定。如果在某侧设置新中位后中止，请先重新完成该侧标定再使用。

```bash
umi-calibrate
```

需要查看标定反馈时，加 `--log`。工具每半秒检查一次，首次读取、状态或提示变化、位置相对上次显示变化至少 4 步时才输出。负载和电流随状态一起显示，单独波动时不会新增状态行。

```bash
umi-calibrate --log
```

如果已用 `sg dialout -c 'umi-id'` 扫描成功，并且还在同一个旧终端里，可以用下面的命令启动标定。输出仍是 `~/umi_gripper/gripper_config.json`，以本教程的目录为准。

```bash
sg dialout -c 'umi-calibrate --log'
```

1. 按 `Enter` 扫描。提示选择通道 `0` 时，输入左侧舵机在列表中的序号。提示选择通道 `1` 时，输入右侧的序号。即使舵机 ID 是 `0`，也要输入它前面的列表序号，例如列表显示 `1. ... ID 0` 就输入 `1`。菜单项 `0` 用于留空该通道。
2. 工具设置单圈绝对位置模式和舵机中位。依次标定闭合、中立、张开三个位置。每个位置用 `W` / `S` 步进，按 `Enter` 确认，按 `Q` 取消本次标定。
3. 三点完成后，输入 `q` 并按 `Enter` 可跳过夹紧力度测试。如需试夹，按 `Enter` 进入，放入测试物后再次按 `Enter` 闭合。可用 `W` / `S` 调整力度，按 `Enter` 确认当前力度，按 `Q` 放弃试夹。
4. 看到对应通道已保存的提示后，查看生成的 JSON。

```bash
python -m json.tool "$HOME/umi_gripper/gripper_config.json"
```

如果驱动板有 `/dev/serial/by-id/` 链接，可以用下面的命令查看链接并打开配置。将对应槽位的 `serial_port` 改成该链接的完整路径。在 `nano` 中按 `Ctrl+O`、`Enter` 保存，再按 `Ctrl+X` 退出。

```bash
ls -l /dev/serial/by-id/
nano "$HOME/umi_gripper/gripper_config.json"
python -m json.tool "$HOME/umi_gripper/gripper_config.json"
```

## 交互试跑

```bash
umi-demo
```

`umi-demo` 会列出已标定的夹爪及其舵机 ID。先按列表选择一只，接下来每行只输入开度。`0` 为全闭，`1` 为全开。输入 `s` 返回夹爪列表，输入 `q` 退出。

例如左夹爪的舵机 ID 是 `1` 时，依次输入以下内容，每行后按 `Enter`。第一行选中左夹爪，接下来依次打开到 80%、全开、全闭。

```text
1
0.8
1
0
q
```

## 应用 API

标定完成后，应用可通过 `umi_gripper.Gripper` 控制夹爪。[HAL Python API](api.md)提供单夹爪和双夹爪示例、接口说明及故障处理流程。

## 配置字段

JSON 有两个标定槽，`servos[0]` 对应左侧，`servos[1]` 对应右侧。空槽填 `null`。

| 字段 | 用途 |
| --- | --- |
| `serial_port`、`servo_id`（0–253）、`baudrate`（9600–1000000） | 读，用来开串口 |
| `closed_position_steps`、`open_position_steps`（0–4095） | 读。`close_01` 0 → 张开步数，1 → 闭合步数 |
| `neutral_position_steps` | 校验行程，可从 `device_info()` 读取 |
| `speed`（1–3400）、`open_grip_strength` / `close_grip_strength`（1–100）、`status_frequency_hz`（1–100） | 先读 JSON，`Gripper.open()` 的 `settings` 可覆盖同名字段。`speed` 原样写入地址 46，见运行时写入表 |
| `accel`（0–254）、`position_tolerance_steps`（0–128）、`stall_movement_steps`（1–128）、`opening_stall_window_s` / `closing_stall_window_s`（0.05–5 秒）、`overtemp_c`（1–100）、`overtemp_clear_c`（0–99，且低于触发温度）、`overtemp_hold_s`（0–5 秒） | `settings` 可覆盖 JSON。缺省值为加速度 50、到位 12 步、堵住 3 步、张开堵转窗 0.5 秒、闭合堵转窗 0.2 秒、过温 70℃ 持续 0.5 秒、65℃ 解除。`accel` 为 0 时使用舵机最大加速度 |
| `servo_model_number`、`calibration_step_steps`、`motion_timeout_s` | 分别用于标定记录、手动步进和 `wait()` 的默认超时 |

张开和闭合力度分别由 `open_grip_strength` 和 `close_grip_strength` 提供。

## 运行时写入

下表列出 UMI 驱动运行时写入的飞特 STS SRAM 地址。上电时，舵机将 EEPROM 地址 16 的最大扭矩复制到地址 48。常见出厂值为 1000，即堵转扭矩的 100%。

| 地址 | 何时写 | 值 |
| --- | --- | --- |
| 40 扭矩开关 | 打开通道时写一次 | `1`（使能）。关闭会话后舵机仍处于使能状态 |
| 48 转矩限制，2 字节，单位 0.1% | 打开时按闭合力度写一次；之后只有力度变化才再写 | `力度 × 10`。40 → 400，即堵转扭矩的 40%。张开用 `open_grip_strength`，闭合用 `close_grip_strength` |
| 41 加速度 | 每次 `WritePosEx` | 配置里的 `accel`。单位 8.7°/s²。写 0 表示最大加速度 |
| 42–43 目标位置 | 下发、到位、停止 | 写入目标步数。到位或停止时写入当前步数 |
| 44–45 PWM 开环速度 | 每次 `WritePosEx` | 固定为 0。此字段仅在运行模式 2 生效 |
| 46–47 运行速度 | 每次 `WritePosEx` | 配置里的 `speed`。相位 bit2 为 0 时单位是 0.732 RPM。驱动写入的数值使方向位 bit15 为 0。配置范围为 1–3400，实际速度受舵机上限约束 |

到达 `position_tolerance_steps` 范围内时，驱动将目标设为当前位置。闭合时，若在 `closing_stall_window_s` 内移动不足 `stall_movement_steps`，状态为 `grasped`，舵机继续以闭合转矩限制保持原目标。张开时，同样的移动阈值配合 `opening_stall_window_s` 判定 `blocked`，驱动将目标设为当前位置。温度达到 `overtemp_c` 并持续 `overtemp_hold_s` 后，驱动停在当前位置并报告 `error`。降至 `overtemp_clear_c` 或更低后可接收新目标。

## 运行时反馈

状态频率由 `status_frequency_hz` 决定。每个周期读取一次地址 56–70，从中取位置、占空比、温度和电流。`Gripper.read()` 返回 `UmiReading`。`close_01` 由当前位置在张开和闭合步数之间换算，`raw_position` 的单位为编码器步数。

未收到舵机回包（SDK `-6`）或串口读写异常时，驱动取消当前目标并报告通信故障。收到有效反馈后，驱动恢复待命，后续动作由应用重新下发。如果拔插后串口句柄仍不可用，关闭实例并在设备恢复后重新 `open()`。可用 [扫描结果](#扫描和核对舵机) 核对配置里的 `serial_port`、`servo_id` 和 `baudrate`。

| 地址 | 含义 | 怎么用 |
| --- | --- | --- |
| 33 模式 | 打开通道时读取一次 | 运行要求为 0（绝对位置） |
| 56–57 当前位置 | 2 字节，0–4095 | `close_01` 和 `raw_position`。超出标定行程（容差 `position_tolerance_steps`，和到位共用）视为无效 |
| 60–61 当前负载 | bit10 是方向，幅值 0–1000，单位 0.1% 驱动电压占空比 | `effort` 保存带符号的原始值。调试脚本显示 `effort / 10`，即带符号的占空比百分比 |
| 69–70 当前电流 | bit15 是方向，单位 6.5 mA | `current` 为安培：先把原始值乘 6.5 后四舍五入成毫安，再除以 1000 |
| 63 当前温度 | 1 字节，℃ | `temperature`。达到 `overtemp_c` 并持续 `overtemp_hold_s` 时停止。降至 `overtemp_clear_c` 或更低时清除过温错误 |
