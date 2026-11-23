# 安装教程

UMI 驱动使用 Python 3.10 或更高版本。安装包通过 `pyproject.toml` 声明串口和舵机 SDK 依赖。

## 独立安装

在 `umi_gripper` 仓库根目录执行下列命令。

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -e .
```

安装后检查包和命令的入口。

```bash
python -I -c 'from umi_gripper import Gripper; print(Gripper.__module__)'
umi-calibrate --help
umi-id --help
umi-demo --help
```

## 安装到现有项目的 Python 环境

激活项目用于运行机械臂应用的 Python 环境，将下面的路径替换为 `umi_gripper` 源码目录。

```bash
python -m pip install -e /path/to/umi_gripper
```

安装后，在应用中用 `Gripper.open()` 加载实机标定并选择夹爪通道。[HAL Python API](api.md)

## 实际设备准备

### 确认 USB 串口

接上驱动板后，在安装了 UMI 的 Python 环境中运行以下命令。通过扩展坞连接 Jetson 时，可以对比插拔驱动板前后的输出。

```bash
lsusb
python -m serial.tools.list_ports -v
ls -l /dev/serial/by-id/
```

- `lsusb` 没有新增设备时，用支持数据传输的 USB 线将驱动板直连主机，排查线缆和扩展坞。
- USB 已识别但没有 `/dev/ttyACM*` 或 `/dev/ttyUSB*` 时，用 `lsusb -t` 查看该设备是否绑定了串口驱动。
- 如果 `/dev/serial/by-id/` 下有指向该串口的链接，运行配置的 `serial_port` 优先填写这个完整路径。`/dev/ttyACM0` 等设备名可能在重新插拔后变动。

### 确认当前终端有串口权限

将示例中的 `/dev/ttyACM0` 换成实际设备名，检查串口属组、账号组和当前会话的组。

```bash
ls -l /dev/ttyACM0
getent group dialout
id -nG
```

如果设备显示 `crw-rw----` 和 `root dialout`，当前会话需要 `dialout` 权限才能打开它。`getent group dialout` 查询系统里的组成员，`id -nG` 显示当前会话实际拥有的组。账号已经出现在 `getent` 输出中，但 `id -nG` 里缺少 `dialout`，说明当前会话还没取得该组权限。此时驱动板可以出现在串口列表里，`umi-id` 和 `umi-calibrate` 仍会跳过打不开的串口，显示「未找到舵机」。

如果账号尚未加入 `dialout`，执行下面的命令。然后重新登录，远程连接时新建 SSH 会话，再运行 `id -nG` 确认组权限已生效。

```bash
sudo usermod -aG dialout "$USER"
```

账号已加入组时，也可以在旧终端激活安装了 UMI 的虚拟环境，用 `sg` 为单条命令临时启用 `dialout`。

```bash
sg dialout -c 'umi-id'
```

确认串口和权限后，检查舵机接线、供电，并将通信板跳线置于 B 位置。标定和试跑时预留夹爪运动空间。下一步见 [扫描和核对舵机](usage.md#扫描和核对舵机)。

## 无硬件验证

从仓库根目录运行 Python 标准库里的 `unittest`。

```bash
python -m unittest discover -s tests -q
```

测试使用 `tests/support.py` 中的模拟总线和脚本化串口反馈，覆盖夹爪控制、共享串口、故障清理和维护命令。[测试辅助代码](../tests/support.py)
