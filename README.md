<div align="center">

# 🎯 征龙凌沧 26 赛季 · 飞镖系统嵌入式代码

**RoboMaster 飞镖（DART）发射 / 换弹控制固件 · STM32H723VGT6**

![MCU](https://img.shields.io/badge/MCU-STM32H723VGT6-03234B?logo=stmicroelectronics&logoColor=white)
![Lang](https://img.shields.io/badge/lang-C%20%2F%20C%2B%2B-A8B9CC?logo=c&logoColor=white)
![Toolchain](https://img.shields.io/badge/toolchain-arm--none--eabi--gcc%20%7C%20MDK--ARM-informational)
![CubeMX](https://img.shields.io/badge/STM32CubeMX-.ioc-blue)
![Status](https://img.shields.io/badge/status-V3.0%20测试中-orange)

</div>

## 简介

征龙凌沧战队 2026 赛季飞镖（M57-DART）的下位机代码。CubeMX 器件为 **STM32H723VGT6**（`STM32H723VGTx`，LQFP100）。`User/` 为 C++，CubeMX 生成的 `Core/` 与 USB 协议栈为 C。

发射机构在 `enable_booster_flag == 1` 后做推进/拉力校准，完成后进入发射状态机；换弹已经并进这条状态机，最多 4 发。云台 yaw 使用 C610，上位机经 MiniPC 下发速度指令。

《版本说明》2026.3.13 记录曾在镖架上打出 4 发，当时仍有小 bug。2026.4.14 把当前树标为 **V3.0 测试代码**：校准复测过，发射和换弹流程还没有再测。

## Demo

仓库里还没有连发视频或状态机图。之后可以放到 `docs/`（目录目前不存在）。

## 特性

- **发射与换弹状态机**：推进、拉力都校准完后切到 `Booster_Control_Type_NORMAL`。发射顺序是 IDLE → 下压锁定 → 换弹（首发跳过）→ 准备 → 待发 → 击发。`Class_FSM_Reload` 不再单独调度。裁判系统允许发射的上升沿写入 `shoot_cmd_token`，IDLE 里消费一次才进入一轮循环。`reload_done_token` 在换弹结束时自增，`last_consumed_reload_done_token` 目前没有被读取。单轮上限是 `kMaxDartCount = 4`。
- **电机与执行器**：GM6020 换弹角度环；C620（版本记录里的 3508）推进电机左右各一，以及同型号拉力电机；拉力计闭环。推进/拉力校准用微动开关找端点，再用 `Linear_Map_Position` 做行程映射。撒放器与三路夹爪是 270° 舵机。`Motor_Reload_Linear` 只出现在注释里，类里没有这个成员，对应 CAN 回调也已注释。
- **yaw 轴**：C610，速度环和角度环都做了初始化。校准第一步向左转，顶到 PB11 微动开关后，按丝杆导程（5 mm/圈，行程上限 350 mm）把编码器角度换成位置。上位机模式是 `Gimbal_Control_Type_MINIPC`，走速度指令。
- **通信**：MiniPC 上位机指令；RS485 接拉力计（《版本说明》2026.1.29 记录曾与串口助手互通）；裁判系统 `Referee_Allow_Shoot` 作为放行信号。USB Device 配置为 HS CDC。
- **微动开关**：推进校准使用 PD7 / PB3，yaw 校准使用 PB11。

## 目录结构

```
Core/ Drivers/ Middlewares/ USB_DEVICE/   CubeMX 生成的 HAL、外设初始化与 USB CDC
DSP/               预编译 CMSIS-DSP（libCMSISDSP.a 与头文件）
                   完整源码另在 Drivers/CMSIS/DSP/
User/
  ├─ Algorithm/    PID、FSM、滤波等
  ├─ Chariot/      发射机构与云台；底盘类仍在工程里并挂了 CAN 回调
  ├─ Congfig/      参数配置（目录名就是这个拼写）
  ├─ Device/       电机、拉力计、裁判系统、MiniPC 等
  ├─ Driver/       CAN / UART / RS485 / USB 等底层封装
  ├─ Interaction/  上位机、裁判系统与遥控器交互
  └─ Task/         任务与外设回调
MDK-ARM/           Keil 工程 ZLLC_2026.uvprojx，输出目录 output/
ZLLC_2026.ioc      CubeMX 工程
STM32H723VGTx_FLASH.ld
startup_stm32h723xx.s
```

`Src/` 下还有两份 CMSIS 数学库（`arm_cortexM7lfdp_math.lib`、`arm_cortexM7lfsp_math.lib`）。

## 快速开始

**GCC（Makefile）**

`Makefile` 里写了 `PREFIX = arm-none-eabi-`，可以用 `GCC_PATH` 指定工具链目录，`TARGET = Sentry`，`BUILD_DIR = build`，CPU 宏是 `STM32H723xx`。文件在 `LDFLAGS` 标题处截断，没有编译和链接规则。`USER_SOURCES` 仍指向不存在的 `User/Middleware/Algorithm/Src` 与 `User/Middleware/Driver/Src`（实际目录是 `User/Algorithm/Src` 和 `User/Driver/Src`）。因此现在不能靠 `make -j` 得到 `build/Sentry.hex`。链接脚本 `STM32H723VGTx_FLASH.ld` 在仓库根目录，但 Makefile 没有引用它。

**Keil MDK**

打开 `MDK-ARM/ZLLC_2026.uvprojx`。器件 `STM32H723VGTx`，输出名 `ZLLC_2026`，输出目录 `MDK-ARM/output/`。

**调试**

- `stm32h723vg_jlink.txt`：器件 `STM32H723VG`，下载 `./build/Sentry.hex`。这个路径对应上面的 GCC 目标名，当前 Makefile 产生不了该文件。
- `test.jdebug`：Ozone 工程，器件 `STM32H723VG`，打开 `MDK-ARM/output/ZLLC_2026.axf`。工程里的 SVD 是本机绝对路径；仓库根目录另有 `STM32H723.svd`。
- 进入 debug run 后，`enable_booster_flag` 默认为 0。把它置为 1 之后，推进/拉力校准才会跑。

## 文档

- [版本说明.md](版本说明.md)：逐版本调试记录与已知问题
- [技术日志.md](技术日志.md)

## License

仓库根目录没有项目级开源协议。`Drivers/`、`Middlewares/` 里的 HAL、CMSIS、USB 协议栈各自带有上游许可证。本仓库协议暂未指定。
