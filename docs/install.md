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
python -I -c 'from umi_gripper import UmiGripper; print(UmiGripper.__module__)'
umi-calibrate --help
umi-id --help
umi-demo --help
```

## 安装到 wholebody 环境

在 wholebody 仓库根目录执行。将 `/path/to/python` 换为启动 wholebody 所用的解释器。

```bash
/path/to/python -m pip install -e ../umi_gripper
```

外设配置和运行方式见 [使用教程](usage.md#接入-wholebody)。

## 实际设备准备

首次标定和改 ID 前确认舵机接线、供电和通信板的跳线位置。维护工具要求跳线处于 B 位置。运行用户需要访问对应 USB 串口。校准和试跑时预留夹爪运动空间。

## 无硬件验证

在本仓环境安装 pytest，从仓库根目录运行测试。

```bash
python -m pip install pytest
python -m pytest -q tests
```

测试通过 `FakeServoBus` 或脚本化串口反馈检查控制与故障清理。
