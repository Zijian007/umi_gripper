# UMI 总线舵机夹爪

`umi_gripper` 是飞特 STS 总线舵机夹爪的硬件抽象层（HAL）。它支持单夹爪和双夹爪，提供标定工具和统一的 Python 控制接口，供不同的机械臂应用使用。驱动负责串口通信、位置换算和运动保护。

- [安装教程](docs/install.md)介绍 Python 环境、串口准备和无硬件验证。
- [使用教程](docs/usage.md)介绍标定、改 ID 和交互试跑。
- [HAL Python API](docs/api.md)介绍应用如何控制已标定的夹爪、读取反馈和处理故障。

## 试跑夹爪

激活装有本包的虚拟环境后，运行 `umi-demo`，按列表选一次舵机 ID。接下来每次只输入开度，`0` 全闭，`1` 全开，`0.8` 表示 80%。输入 `s` 换夹爪，输入 `q` 退出。

安装本包时，Python 安装工具会按 `pyproject.toml` 生成 `umi-demo` 命令，实际入口是 `demo.py` 里的 `cli()`。在仓库根目录，也可以用已装好依赖的 Python 直接运行 `python -m umi_gripper.cli.demo`。两个入口都支持 `--help`。[命令注册](pyproject.toml#L16) · [试跑教程](docs/usage.md#交互试跑)

| 命令 | 用途 | 入口 |
| --- | --- | --- |
| `umi-calibrate` | 采集闭合、中立和张开位置，试夹后保存标定 | [calibrate.py](umi_gripper/cli/calibrate.py) |
| `umi-id` | 扫描、核对和修改舵机 ID | [identifier.py](umi_gripper/cli/identifier.py) |
| `umi-demo` | 加载标定文件并试跑指定开度 | [demo.py](umi_gripper/cli/demo.py) |

标定和改 ID 共用舵机扫描及选择菜单。[discovery.py](umi_gripper/cli/discovery.py)

`umi-calibrate` 默认把实机标定保存到仓库根目录的 `gripper_config.json`。`umi-demo` 和 `Gripper.open(channel=...)` 共用这个默认路径。归档中的 JSON 记录了之前的设备标定。[历史标定记录](docs/worklog/records/gripper_config.historical.json)

应用通过 `from umi_gripper import Gripper` 控制夹爪。`session.py` 管理共享串口和控制循环，`config.py` 负责配置读写、标定校验和位置换算。[夹爪接口](umi_gripper/gripper.py) · [串口会话](umi_gripper/session.py) · [配置与标定](umi_gripper/config.py)
