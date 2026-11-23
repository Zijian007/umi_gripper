# UMI 夹爪 HAL Python API

夹爪已完成标定，应用可以直接用 `umi_gripper.Gripper` 控制它。把 `umi_gripper` 装在应用运行的 Python 环境中，在应用配置里保存实机标定 JSON 的路径和通道号。一个 `Gripper` 实例控制一只夹爪。[安装方法](install.md#安装到现有项目的-python-环境)

## 打开和控制一只夹爪

把示例中的路径和通道换成实机绑定。`channel` 是标定 JSON 的槽位，`0` 对应 `servos[0]`，`1` 对应 `servos[1]`。舵机 ID 由所选槽位里的 `servo_id` 决定。

```python
from pathlib import Path

from umi_gripper import Gripper, UmiError

calibration_path = Path("/path/to/robot/gripper_config.json")
channel = 0

info = Gripper.validate_config(config_path=calibration_path, channel=channel)
print("舵机 ID", info["calibration"]["servo_id"])

with Gripper.open(config_path=calibration_path, channel=channel) as gripper:
    gripper.set_target(0.0)
    reading = gripper.wait(timeout=2.0)
    if reading.state != "arrived" or not reading.feedback_valid:
        raise UmiError(f"夹爪未到达全开位置 ({reading.state}) {reading.message}")
    print("实际闭合比例", reading.close_01)
```

`close_01` 表示闭合比例。`0.0` 全开，`1.0` 全闭，`0.2` 表示闭合 20%。如果应用内部使用开度比例，传入 `1.0 - opening_01`。

`validate_config()` 可在应用启动前读取并校验标定和运行参数，返回设备信息。`open()` 会核对舵机通信和初始位置，并使能扭矩。两个方法都可接收 `settings`，为此次打开覆盖速度、力度、反馈频率和运动保护参数。[可覆盖的配置字段](usage.md#配置字段)

## 在应用中管理设备

在应用启动时打开夹爪，把实例保留到退出。事件处理或控制循环通过 `set_target()` 提交目标，随后用 `read()` 获取缓存反馈。需要等动作结果的顺序任务用 `wait()`。周期控制循环可根据 `state` 推进自己的流程。

```python
from pathlib import Path

from umi_gripper import Gripper

calibration_path = Path("/path/to/robot/gripper_config.json")

with (
    Gripper.open(config_path=calibration_path, channel=0) as left,
    Gripper.open(config_path=calibration_path, channel=1) as right,
):
    left.set_target(0.0)
    right.set_target(0.0)
    for side, gripper in (("left", left), ("right", right)):
        result = gripper.wait(timeout=2.0)
        print(side, result.state, result.close_01)
```

同一进程内，同一物理串口上的夹爪共用轮询线程，也必须使用相同波特率。一个串口上的每个舵机 ID 同时只能有一个控制实例。退出 `with` 会关闭实例。关闭同一串口的最后一个实例时，HAL 会释放串口。关闭后舵机仍保持扭矩使能。

## 公开接口

| 接口 | 行为 |
| --- | --- |
| `Gripper.validate_config(channel=..., config_path=..., settings=...)` | 离线校验所选槽位，返回与 `device_info()` 相同结构的字典。 |
| `Gripper.open(channel=..., config_path=..., settings=...)` | 打开一只已标定的夹爪，返回支持 `with` 的 `Gripper`。 |
| `set_target(close_01, retry=False)` | 提交目标后返回。重复提交换算后相同的目标会保留当前动作和结果。动作结束后可以用 `retry=True` 重新执行相同目标。 |
| `read()` | 返回最近一次缓存的 `UmiReading`。同一份反馈的采样时间保持不变。 |
| `wait(timeout=None)` | 等到状态离开 `moving` 并返回 `UmiReading`。超时抛出 `UmiError`，应用可按需调用 `stop()`。省略超时时使用 `motion_timeout_s`，缺省为 30 秒。 |
| `stop()` | 请求停在当前位置并等待结果。停止后可重新提交同一目标。 |
| `close()` | 关闭控制实例。支持重复调用，正常退出也可以通过 `with` 自动关闭。 |
| `device_info()` | 返回 `channel`、`position_unit`、`calibration` 和 `runtime`。`calibration` 含串口、舵机 ID、波特率和标定点，`runtime` 含生效的驱动参数。 |

## 读取动作结果和反馈

| `UmiReading.state` | 含义 |
| --- | --- |
| `ready` | 待命，例如刚完成初始化、主动停止或通信恢复。 |
| `moving` | 目标执行中，或正在停到当前位置。 |
| `arrived` | 进入标定配置的目标位置容差。 |
| `grasped` | 闭合时达到停滞判据，HAL 维持闭合目标和闭合力度。 |
| `blocked` | 张开时达到停滞判据，HAL 改为保持当前位置。 |
| `error` | 发生故障。读取 `fault`、`fault_code` 和 `message` 获取细节。 |

应用可通过 `feedback_valid` 检查位置反馈有效性，并用 `time.monotonic() - reading.sampled_at_mono_s` 得到反馈距今的秒数。有效期由应用按控制周期决定。反馈失效时 `close_01` 为 `None`，`raw_position` 等字段可能保留上一次读数供诊断。

`raw_position` 的单位是编码器步数，`current` 是安培，`temperature` 是摄氏度，`effort` 是带符号的 PWM 占空比计数，每计数为 0.1%。

## 处理故障和退出

- `wait()` 返回动作结果时，应用按场景判断是否成功。全开通常要求 `arrived`，夹持任务可根据实测位置和应用条件接受 `grasped`。
- 通信故障时，HAL 取消执行中的目标。收到新的有效反馈后恢复待命，由应用决定何时重新下发目标。USB 拔插后若原串口句柄仍不可用，关闭原实例并在设备恢复后重新打开。
- `fault` 为 `communication`、`position`、`overtemperature` 或 `None`。`fault_code` 有值时可用于记录 SDK 错误码。配置、通信或超时等调用错误可能抛出 `UmiError`，非法开度或超时参数会抛出 `ValueError`。
- 如果 `close()` 失败，用原实例重试。若 `open()` 抛出的 `UmiError` 带有 `resource`，应用可以通过 `exc.resource.close()` 完成清理。
