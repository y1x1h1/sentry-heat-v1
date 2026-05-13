# 哨兵自瞄 · 热量反馈版 v1

基于 STM32F427IIH6 的 RoboMaster 哨兵机器人电控系统，搭载自瞄算法与热量管理。

## 上位机配套

本工程参考 **[君瞄 (rm_vision)](https://gitlab.com/rm_vision)** 自瞄上位机协议设计，视觉通信协议与其完全匹配：

- **下位机 → 上位机** (200Hz)：发送 IMU 姿态（roll/pitch/yaw），上位机据此进行 PnP 解算
- **上位机 → 下位机**：接收目标三维坐标 (x, y, z) 与跟踪状态，驱动云台锁定

## 硬件平台

| 组件 | 型号 | 
|------|------|
| 主控 | STM32F427IIH6 @ 168MHz |
| IMU | SPI 陀螺仪 + 加速度计 (SPI5) |
| Yaw 电机 | RM 6020 |
| Pitch 电机 | RM 6020 |
| 摩擦轮 ×2 | RM 3508 |
| 拨弹电机 | RM 2006 (36:1 减速比) |
| 视觉通信 | USART6 DMA |
| 裁判系统 | USART1 (100000bps, 9-bit Even) |
| 操作手遥控 | UART8 |

## 自瞄链路

```
操作手 UART8 指令 ──→ 模式切换 ──→ 拨弹许可
                           │
IMU (SPI) ──→ Mahony 姿态解算 (1kHz)
                │
                ├──→ 200Hz 发给视觉上位机 (USART6)
                │
上位机 PnP解算 ──→ 目标 (x,y,z) DMA 接收 (USART6)
                │
                ├──→ 弹道解算 (Solve_Pitch_Control)
                │
                └──→ 云台 PID (角度环 → 速度环) → CAN 电机
                              │
                        热量检查 ←── 裁判系统热量 (USART1)
                              │
                        拨弹控制 (位置环 → 速度环) → CAN 电机
```

### 1. 姿态解算 (Mahony AHRS)

- 运行于 TIM6 1kHz 中断
- 读取 IMU 原始角速度/加速度，经 Mahony 互补滤波解算欧拉角
- 轴向已做映射修正 (yaw 轴旋转 90°)
- 解算结果 `imu_yaw / imu_pitch / imu_roll` 供视觉发送和云台控制使用

### 2. 视觉通信协议

**发送帧** (28 bytes, 200Hz):
| 字段 | 类型 | 说明 |
|------|------|------|
| header | uint8 | 0x5A |
| detect_color | uint1 | 敌方颜色 |
| roll/pitch/yaw | float×3 | IMU 绝对姿态角 (rad) |
| aim_x/y/z | float×3 | 预留 |
| checksum | uint16 | CRC16 |

**接收帧** (48 bytes):
| 字段 | 类型 | 说明 |
|------|------|------|
| header | uint8 | 0xA5 |
| tracking | uint1 | 0=丢失, 1=锁定 |
| id/armors_num | uint3×2 | 目标 ID 与装甲板数量 |
| x, y, z | float×3 | 目标在枪口坐标系下的三维坐标 (m) |
| yaw | float | 目标 yaw 角 |
| vx, vy, vz | float×3 | 目标速度 |
| v_yaw | float | 目标角速度 |
| r1, r2, dz | float×3 | 装甲板参数 |
| checksum | uint16 | CRC16 |

### 3. 弹道解算

`Solve_Pitch_Control(x, y, v)` 使用迭代法求解弹道补偿角：

- **输入**：水平距离 `x`、垂直高度差 `y`、弹速 `v` (18m/s)
- **算法**：初始角 = atan2(y, x)，迭代修正重力下坠 (最多 20 次)
- **输出**：Pitch 抬枪角度 (rad)
- 物理模型：`y = v·sinθ·t - 0.5·g·t²`，忽略空气阻力
- 精度：误差 < 0.001m

### 4. 云台控制

**状态机**：

| 状态 | 触发条件 | 行为 |
|------|----------|------|
| INIT | 上电复位 | 1.5s 后自动进入 NORMAL |
| NORMAL | 默认 | tracking=0 巡航扫描 / tracking=1 锁定跟踪 |
| CHECK_IN | UART8 mode=1 | 云台归中，不响应视觉，等待检录 |

**巡航模式** (tracking=0)：
- Yaw 范围：-60° ~ +20°（以中心为基准）
- Pitch 范围：机械限位内往复
- 平滑低通滤波 (α=0.10)

**跟踪模式** (tracking=1)：
- Yaw 目标 = 当前电机位置 + atan2(y, x) 相对偏移
- Pitch 目标 = 弹道解算角度 + 中心偏移
- 一阶低通滤波 (α=0.70)
- 锁定瞬间有 300ms 稳定期 (lock_settle_timer)
- 重力前馈补偿：`1300 × cos(imu_pitch)`
- 角度环 Kp 动态切换（巡航 0.08 / 锁定 0.8）

**级联 PID 结构**：
```
角度环 (P) → 速度环 (PID) → CAN 电流指令
```

### 5. 射击与热量管理

遵循 **RMUL 2026 哨兵规则**：

- 热量上限：260
- 每发 17mm 弹丸：10 热量
- 安全余量：30 (预判缓冲)
- 触发条件：`当前热量 + 10 + 30 < 260`

**拨弹控制**：
- M2006 电机 + 36:1 减速比，拨盘 7 孔/圈
- 每发弹丸对应电机角度：`36 × 360 / 7 ≈ 1851.43°`
- 位置环增量式控制（角度到位 → 减速 → 到位累计下一发）
- 堵转保护：400ms 未到位强制卸载电流，900ms 后重试

**摩擦轮**：
- 双 3508 电机对转，目标转速 6500 RPM
- 速度闭环 PID

### 6. 热量反馈闭环

裁判系统通过 USART1 实时下发当前热量值 `receiver_heat`，射击任务在每次拨弹前检查热量余量。热量不足时停止拨弹，摩擦轮保持转速；热量恢复后自动继续射击。

## 目录结构

```
├── Core/           STM32CubeMX 生成 HAL 代码
│   ├── Inc/        外设头文件 (can/dma/gpio/spi/usart)
│   └── Src/        外设源文件 + main.c + 中断服务
├── Drivers/        CMSIS + STM32F4 HAL 库
├── MDK-ARM/
│   ├── algo/       算法层
│   │   ├── mahony.c     Mahony AHRS 姿态解算
│   │   ├── pid.c        PID 控制器
│   │   ├── solve_trajectory.c  弹道解算 (迭代法)
│   │   └── crc.c        CRC16 校验
│   ├── app/        应用层
│   │   ├── gimbal.c     云台状态机 & 双环 PID
│   │   ├── shoot.c      射击控制 & 热量管理
│   │   └── vision.c     视觉通信协议
│   ├── bsp/        板级驱动
│   │   ├── bsp_can.c    CAN 电机总线
│   │   ├── bsp_imu.c    SPI IMU 读取
│   │   ├── bsp_serial.c USART6 视觉 DMA 初始化
│   │   ├── bsp_uart8.c  操作手遥控 & 裁判数据解析
│   │   └── bsp_debug.c  调试波形发送 (Vofa+)
│   └── startup_stm32f427xx.s  启动文件
└── sentry2.ioc     CubeMX 工程文件
```

## 编译

- IDE: Keil MDK-ARM V5.32+
- 工具链: ARM Compiler 5/6
- CubeMX 版本: 6.17.0
- HAL 库: STM32Cube FW_F4 V1.28.3

## 备注

- 带热量反馈的第一版自瞄，尚未上车实测
- 云台通信看门狗：500ms 未收到 UART8 数据则退出检录模式
- 调试支持 Vofa+ 波形查看（UART7, 460800bps）
