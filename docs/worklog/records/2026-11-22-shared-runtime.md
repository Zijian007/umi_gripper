# 2026-11-22 UMI 夹爪运行时重构

| 项 | 值 |
|---|---|
| 作者 | Codex |
| 机器 | `g2-dc` |
| 分支 @ 提交 | umi_gripper `main` @ `dd40d10`、wholebody `refactor/extract-robot-hal` @ `8c9ca8c`、wholebody `feat/episode-recorder` @ `ca72b2f`，均包含工作区改动 |
| 环境 | Linux aarch64，Python 3.10.12，PySerial 3.5，FTServo SDK 2.0.0 |
| 构建 | `umi_gripper-0.1.2-py3-none-any.whl`，临时目标目录安装 |
| 结果 | UMI 30 通过，wholebody 重构分支 70 通过、录制分支 45 通过。最终离线测试 0 失败、0 跳过。真机未运行 |
| 类型 | 机器实测（fake 与模拟录制） |

`Gripper` 兼容入口、`UmiGripper` 和标定程序共用运动循环、串口和位置换算。驱动测试集中到 `umi_gripper/tests`，两个 wholebody 分支保留应用适配测试。配置 JSON 原有格式继续由兼容入口读取。关闭失败会保留资源供调用方重试。

wheel 的 SHA-256 为 `6111feaba8d82f38d207842d0b3600e49465a0055c1656cc3b7a65b04ade6d51`。在源码目录外检查包导入、兼容导入以及 `umi-calibrate --help`、`umi-id --help`、`umi-demo --help`。两个 wholebody 分支使用该 wheel 跑相关测试。录制分支的两项进程测试需要绑定本机 Unix 套接字，获准后复跑通过。

安装步骤和维护用法分见 [安装教程](../../install.md)、[使用教程](../../usage.md)。域控真机验收见 [待办](../todo.md)。
