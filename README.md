# 扫地机器人（Sweeping Robot）—— STM32F103 + FreeRTOS
<img width="1195" height="599" alt="8c4cb302e992f6568440545fde6afbf6" src="https://github.com/user-attachments/assets/2d9d17ad-4b53-473e-8623-151f92b61bc1" />

基于 **STM32F103C8T6** 的扫地机器人固件。硬件包含 TB6612 双路电机驱动、HC-SR04 超声波避障、
R140 吸尘马达，以及蓝牙 / 离线语音 / 串口屏三路人机交互通道。

代码最初是裸机（前后台）架构：全部逻辑塞在 `main()` 的 `while(1)` 里，用 `HAL_Delay()`
阻塞等待；现在已重构为 **FreeRTOS 多任务调度**，用任务 + 队列 + 信号量把测距、控制、
通信解耦。

> 没有硬件也可以玩：[`simulation/index.html`](simulation/index.html) 是零依赖的网页仿真，
> 双击打开即可用键盘操控机器人在房间里清扫灰尘（尾部风扇联动开启），详见第 10 节。

---

## 1. 快速上手

| 项目 | 说明 |
| --- | --- |
| 主控 | STM32F103C8T6（Cortex-M3，72MHz，64KB Flash / 20KB RAM） |
| 开发环境 | Keil MDK-ARM（ARM Compiler 5，MicroLIB） |
| 工程文件 | `MDK-ARM/design.uvprojx` |
| RTOS | FreeRTOS Kernel V10.5.1（`Middlewares/FreeRTOS`） |
| 移植层 | `portable/RVDS/ARM_CM3`（Keil ARMCC 专用）+ `heap_4` 内存管理 |

打开 `MDK-ARM/design.uvprojx` 直接编译下载即可。

> 注意：FreeRTOS 的源文件和头文件搜索路径已经加进 Keil 工程，不需要再手动添加。
> 工程里 `Application/User/Core` 组新增了 4 个应用层源文件，`Middlewares/FreeRTOS`
> 组新增了 5 个内核源文件。

---

## 2. 硬件资源分配

| 外设 | 引脚 / 定时器 | 用途 |
| --- | --- | --- |
| USART1 | PA9 / PA10，9600bps | HC-08 蓝牙模块（同时也作为调试控制台，`printf` 输出） |
| USART2 | PA2 / PA3，9600bps | SU-03T 离线语音模块 |
| USART3 | PB10 / PB11，9600bps | 串口触摸屏 |
| TIM3_CH1 | PA6 | 左轮 PWM（占空比 0~100） |
| TIM3_CH2 | PA7 | 右轮 PWM |
| TIM3_CH3 | PB0 | 吸尘器（吸尘马达）PWM |
| TIM3_CH4 | PB1 | 扫把（预留，未启用） |
| GPIO | PB3 / PB4 | 左轮方向（AIN1 / AIN2） |
| GPIO | PB5 / PB6 | 右轮方向（BIN1 / BIN2） |
| GPIO | PB12 / PB13 | 吸尘器方向 |
| GPIO | PB7 | HC-SR04 Trig（触发） |
| TIM4_CH3 | PB8 | HC-SR04 Echo（输入捕获，1MHz 计数 = 1us 分辨率） |
| TIM2 | — | 已配置但不再启动中断（见第 6 节） |

TIM3 的分频为 720-1、周期 100-1，即 PWM 频率 1kHz，`CCR = 0~100` 对应 0~100% 占空比。

---

## 3. FreeRTOS 任务架构

4 个任务，优先级数值越大越高（`configMAX_PRIORITIES = 5`）：

| 任务 | 优先级 | 周期 | 栈 | 职责 |
| --- | --- | --- | --- | --- |
| `control` | 4（最高） | 20ms | 160 word | **唯一的执行器任务**：解析指令队列、运行自动避障状态机、驱动电机与吸尘器 |
| `sensor` | 3 | 100ms | 128 word | 触发 HC-SR04，等待回波信号量，换算距离并写入距离信箱 |
| `comm` | 2 | 事件驱动 | 192 word | 从指令队列取字节，校验后转发给控制任务 |
| `report` | 1（最低） | 1000ms | 192 word | 通过 USART1 周期上报最新距离 |

设计要点：

- **执行器只有一个“主人”**：所有电机 / 吸尘器操作都发生在 `control` 任务里，
  不会出现两个任务同时改 PWM 或方向引脚的情况（裸机版本是在串口中断里直接操作电机）。
- **中断只做“搬运”**：串口中断只往队列投递 1 个字节，回波中断只给信号量，
  不做任何耗时操作，中断响应快，也不会和任务里的电机操作抢资源。
- **测距与上报解耦**：测距 100ms 一次（避障反应更快），上报仍保持 1s 一次，互不影响。

### 任务间通信

```
        USART1/2/3 接收中断
                │  xQueueSendFromISR(1 字节)
                ▼
          g_cmdQueue (深度 8)
                │  通信任务：校验指令
                ▼
          g_ctrlQueue (深度 8)
                │  控制任务：执行动作 / 状态机
                ▼
             电机 / 吸尘器

        TIM4 捕获中断 (Echo 下降沿)
                │  xSemaphoreGiveFromISR
                ▼
            g_echoSem ──► 测距任务：读取回波时间 → 换算距离
                                  │  xQueueOverwrite(信箱，深度 1)
                                  ▼
                             g_distQueue ──► 控制任务（避障判断）
                                        └──► 上报任务（1s 打印）
```

- `g_distQueue` 是**深度 1 的信箱**：始终只保留最新一次测距结果，控制任务用
  `xQueuePeek()` 读取（只读不取），不会因为消费不及时而产生旧数据堆积。
- 上报任务与控制任务共享距离数据，但两者互不阻塞。

### 自动模式（避障）状态机

裸机版本用 `HAL_Delay(3000)` / `HAL_Delay(5000)` 等待，等待期间主循环停摆，
连“停止”指令都响应不了。现在改成 20ms 周期的状态机：

```
MODE_AUTO_FORWARD   中速前进，持续读距离
      │ 距离 <= 25cm
      ▼
 MODE_AUTO_BACKUP   快速后退 3s
      │ 3s 到
      ▼
 MODE_AUTO_TURN     左转 5s
      │ 5s 到
      ▼
（回到 MODE_AUTO_FORWARD）
```

任何手动指令都会让机器人退出自动模式并立即执行（与裸机版本行为一致），
但**等待期间依然可以随时响应新指令**。

---

## 4. 串口指令协议

三路串口共用同一套单字节指令（与原工程完全一致）：

| 指令 | 值 | 功能 |
| --- | --- | --- |
| 停止 | `0x01` | 停机、退出自动模式、关吸尘器 |
| 前进 | `0x02` | 方向向前（速度保持当前档位） |
| 后退 | `0x03` | 方向向后 |
| 左转 | `0x04` | 差速左转（左 40 / 右 100） |
| 右转 | `0x05` | 差速右转（左 100 / 右 40） |
| 快速 | `0x06` | 速度 100 |
| 中速 | `0x07` | 速度 80 |
| 慢速 | `0x08` | 速度 60 |
| 吸尘器 | `0x09` | 打开吸尘器（PWM 100） |
| 自动模式 | `0x10` | 自动前进 + 超声波避障 |

把 `app_config.h` 里的 `APP_LOG_COMMANDS` 改成 `1`，通信任务会把每条收到的指令
打印到 USART1，方便联调定位。

---

## 5. 中断优先级（很重要）

Cortex-M3 上，**允许调用 `...FromISR()` 内核接口的中断，其优先级数值必须 >= 5**
（`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`）。本工程的设置：

| 中断 | 优先级 | 说明 |
| --- | --- | --- |
| SysTick | 15（最低） | 由 FreeRTOS 配置，同时喂 HAL 时基 |
| SVC / PendSV | 15 | FreeRTOS 上下文切换 |
| TIM4（回波捕获） | 5 | 回调里 `xSemaphoreGiveFromISR()` |
| USART1 / 2 / 3 | 5 | 回调里 `xQueueSendFromISR()` |
| TIM2 | 0（未启用中断） | 不调用任何内核接口，可保持高优先级 |

三个内核异常的接线方式：

- `SVC_Handler` / `PendSV_Handler` 由 FreeRTOS 移植层提供，
  在 `FreeRTOSConfig.h` 里通过宏映射到 CMSIS 标准向量名；
- `SysTick_Handler` 保留在 `stm32f1xx_it.c` 中：
  `HAL_IncTick()` → `xPortSysTickHandler()`，
  这样 HAL 的 `HAL_GetTick()` / `HAL_Delay()` 时基依然有效（调度器启动前不会调用内核接口）。

---

## 6. 相比裸机版本修复的问题

| # | 裸机版本的问题 | 现在做法 |
| --- | --- | --- |
| 1 | 三路串口**共用同一个** `RxBuff1`，任意一路收到数据都会把三路接收重新挂载一遍 → 丢字节、重复触发（现象：**要按两次停止才停**） | 每路串口独立的 `s_rx_byte[i]`，且只重新挂载自己这一路 |
| 2 | 串口中断里**直接调用电机函数**（大量 GPIO 操作），中断处理时间长 | 中断只投递字节到队列，动作交给 `control` 任务 |
| 3 | `HAL_UART_Transmit()` 阻塞发送会长时间占用 `huart->Lock`，期间中断里 `HAL_UART_Receive_IT()` 返回 `HAL_BUSY` → **该串口永久收不到数据** | 控制台改用寄存器级轮询发送（`App_ConsoleSend()`），不占用 HAL 锁 |
| 4 | 自动模式用 `HAL_Delay(3000/5000)` 阻塞，期间**无法响应任何指令** | 状态机 + 时间戳，20ms 周期，随时可响应 |
| 5 | `wave_flag` 用完不清零，判断条件只要 `flag_time==0` 就成立 → 避障动作**反复触发** | 用信号量同步，一次回波只触发一次判断，状态机保证每个动作只进入一次 |
| 6 | TIM2 每 10us 中断一次（100kHz），**约占 20%~30% CPU** | 改用 TIM4 捕获时间戳直接相减，1us 分辨率，**零中断开销** |
| 7 | 超声波触发脉冲用软件空循环延时（`-O3` 下可能被优化掉） | 用 TIM4 的 1MHz 计数做延时，精度 1us 且不会被优化 |
| 8 | 没有故障兜底：栈溢出 / 内存分配失败会直接跑飞 | 提供 `vApplicationStackOverflowHook` / `vApplicationMallocFailedHook`，先把电机停下来再停机 |
| 9 | 串口一旦发生过载/噪声错误，HAL 会中止本次接收且不再挂载 → 该路串口**永久失效** | 实现 `HAL_UART_ErrorCallback()`，出错后自动重新挂载接收 |

> 行为兼容性：指令表、PWM 占空比、避障阈值（25cm）、后退 3s + 左转 5s 的动作序列、
> 1s 上报距离的输出格式都与原工程保持一致，方便继续用原来的手机 APP / 语音模块联调。
> 上电提示 `"OK"` 仍按原工程发往 USART2。

---

## 7. 目录结构

```
design/
├── Core/
│   ├── Inc/
│   │   ├── app_config.h        # 指令码、任务参数、避障阈值（统一配置入口）
│   │   ├── app_rtos.h          # RTOS 对象句柄与初始化接口
│   │   ├── app_sensor.h        # 超声波测距接口
│   │   ├── app_control.h       # 运动控制 / 工作模式接口
│   │   ├── app_comm.h          # 通信与上报任务接口
│   │   ├── FreeRTOSConfig.h    # FreeRTOS 内核配置
│   │   ├── main.h / tim.h / usart.h / gpio.h / motor.h / stm32f1xx_it.h
│   │   └── stm32f1xx_hal_conf.h
│   └── Src/
│       ├── main.c              # 只做硬件初始化 + 启动调度器
│       ├── app_rtos.c          # 内核对象创建、任务创建、控制台输出、故障钩子
│       ├── app_sensor.c        # 测距任务 + TIM4 回波捕获回调
│       ├── app_control.c       # 控制任务 + 自动模式状态机
│       ├── app_comm.c          # 通信任务 + 上报任务 + 串口接收回调
│       ├── motor.c             # 电机驱动（PWM / 方向 / 吸尘器）
│       ├── tim.c / usart.c / gpio.c / stm32f1xx_it.c / stm32f1xx_hal_msp.c
│       └── system_stm32f1xx.c
├── Middlewares/FreeRTOS/       # FreeRTOS Kernel V10.5.1（tasks/queue/list + RVDS CM3 移植 + heap_4）
├── Drivers/                    # STM32F1 HAL + CMSIS
├── MDK-ARM/                    # Keil 工程（design.uvprojx）
└── simulation/                 # 网页仿真（index.html + sim.js，双击即玩，见第 10 节）
```

---

## 8. RTOS 配置要点（`FreeRTOSConfig.h`）

| 配置 | 值 | 说明 |
| --- | --- | --- |
| `configTICK_RATE_HZ` | 1000 | 1ms 系统节拍 |
| `configMAX_PRIORITIES` | 5 | 4 个任务 + 空闲任务 |
| `configTOTAL_HEAP_SIZE` | 6KB | heap_4，够 4 个任务栈 + 队列 + TCB |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | 栈溢出检测（方法二） |
| `configUSE_MALLOC_FAILED_HOOK` | 1 | 分配失败钩子 |
| `configUSE_TIMERS` | 0 | 不用软件定时器，省 Flash / RAM |
| `configUSE_TICK_HOOK` | 0 | HAL 时基在 `SysTick_Handler` 里直接喂 |

---

## 9. 构建与验证情况

移植完成后用 **Arm GNU Toolchain 13.3.1（arm-none-eabi-gcc，Cortex-M3 / thumb）** 对
`Core/`（含 4 个新任务文件）+ `Drivers/`（HAL）+ `Middlewares/FreeRTOS/` 做了一次完整体
编译与链接验证：

| 检查项 | 结果 |
| --- | --- |
| 编译（`-Wall`） | 全部通过，**工程源码 0 警告** |
| 链接 | 通过（三个内核异常处理函数各只有一份定义，无重复/未定义符号） |
| 代码体积 | `.text` ≈ 20.1KB，`.data` 112B（64KB Flash 上限） |
| RAM 静态占用 | `.bss` ≈ 7.1KB（其中含 6KB FreeRTOS 堆），加上 Keil 启动文件的 1KB 栈，20KB RAM 余量充足 |

> 说明：上面是 GCC 交叉编译的验证结果（用于确认代码可编译、可链接、体积可控）。
> Keil MDK 工程（`design.uvprojx`，ARMCC 5 + MicroLIB）未在本机编译，
> 请在 Keil 里 Build 一次确认；FreeRTOS 移植层已按 Keil 专用目录
> `portable/RVDS/ARM_CM3` 配置好。

**Keil 打开工程时如果中文注释显示乱码**：源码统一保存为 UTF-8，
在 Keil 里选择 `Edit → Configuration → Editor → Encoding: UTF-8` 即可正常显示。

---

## 10. 网页仿真（simulation/，无需硬件）

`simulation/index.html` 是一个**零依赖的浏览器仿真**：双击打开即可玩，也可以把仓库开启
GitHub Pages 后在线访问。机器人在一个带家具（沙发/床/茶几/柜子/书桌）的房间里清扫
随机分布的灰尘，外观按结构图绘制（双驱动轮、中部电池、尾部风扇舱 + 旋转刷盘）。

控制逻辑 1:1 复刻固件：

| 固件 | 仿真中的对应实现 |
| --- | --- |
| 指令协议（`app_config.h` 的 `0x01~0x10`） | 键盘按键直接投递相同指令码，右侧 HUD 显示最近指令 |
| PWM 档位 慢 60 / 中 80 / 快 100，转向差速 40/100（`motor.c`） | 差速运动学按相同占空比解算左右轮速 |
| 自动模式状态机：前进 → 障碍 ≤25cm → 快速后退 3s → 左转 5s（`app_control.c`） | 完全相同的状态名（`manual / auto-forward / auto-backup / auto-turn`）与时序 |
| HC-SR04 测距 100ms 周期、超量程 561cm（`app_sensor.c`） | 车头射线检测墙/家具，HUD 实时显示距离 |
| 1s 周期上报距离（`app_comm.c` 上报任务） | “遥测输出”窗口按 `printf("%d\r\n")` 格式滚动打印距离 |
| 吸尘器（尾部风扇，`besom_run()` PWM 100） | 尾部三叶风扇 + 黄色刷盘旋转动画，行驶即联动开启，吸入尾部刷盘附近的灰尘 |

### 按键映射

| 按键 | 指令码 | 功能 |
| --- | --- | --- |
| `W` / `↑` | `0x02` | 前进 |
| `S` / `↓` | `0x03` | 后退 |
| `A` / `←` | `0x04` | 左转（差速 40/100） |
| `D` / `→` | `0x05` | 右转（差速 100/40） |
| `1` / `2` / `3` | `0x08` / `0x07` / `0x06` | 慢速 / 中速 / 快速 |
| `F` | `0x09` | 吸尘器（尾部风扇）开/关 |
| `G` / `Enter` | `0x10` | 自动模式（避障 + 吸尘） |
| `Space` | `0x01` | 停止（退出自动、关风扇） |
| `R` | — | 重置仿真（重新撒灰尘） |

仿真附加设定（为便于演示，与固件略有差异的两点）：机器人一旦开始行驶（手动或自动）
尾部风扇即联动开启，停止时关闭（固件里吸尘由 `0x09` 单独控制）；键盘按下方向键时若
当前 PWM 为 0 会自动补上最近档位（相当于手机 APP 上“先选档再开车”的合并操作）。

---

## 11. 版本与许可

- FreeRTOS Kernel **V10.5.1**（MIT 许可，见 `Middlewares/FreeRTOS/LICENSE.md`），
  仅取用 `tasks.c`、`queue.c`、`list.c`、`portable/RVDS/ARM_CM3`、`portable/MemMang/heap_4.c`。
- STM32 HAL / CMSIS 遵循 ST 的 BSD-3-Clause 许可。
- 应用层代码（`Core/` 下的 `app_*.c/h`）为本项目实现。
