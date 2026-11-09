# 综合物流分拣智能搬运小车

本工程包含 STM32 主控、ESP32 网页控制、K230 视觉识别程序和 YOLO 模型。

## 工程入口

| 部分 | 位置 | 工具 |
|---|---|---|
| STM32F407VE 主控 | `MDK-ARM/robot.uvprojx` | Keil MDK，ARM Compiler 6 |
| ESP32-WROOM / DevKit | `ESP32/robot_WebControl/robot_WebControl.ino` | Arduino IDE，ESP32 Dev Module |
| ESP32-S3 N16R8 | 同上，使用同一份程序 | Arduino IDE，ESP32S3 Dev Module；设置见下文 |
| K230 视觉 | `K230部署/main.py` | CanMV IDE |
| YOLO 模型 | `K230部署/yolo11n_det_320.kmodel` | YOLO11，320 × 320，24 个输出类别 |
| 网页预览 | `ESP32/网页预览/server.cjs` | Node.js |
| 引脚配置参考 | `robot.ioc` | STM32CubeMX，RTOS 配置以 Keil 工程和 `FreeRTOSConfig.h` 为准 |
| 软件测试 | `tests/` | Python、Node.js、MSVC 主机模拟 |

`Core/` 为主控应用，`HARDWARE/` 为外设驱动，`Drivers/` 为 STM32 HAL/CMSIS。Keil 编译产物和本机缓存不纳入版本管理。

## FreeRTOS 运行结构

STM32 使用 FreeRTOS V10.3.1，1ms 系统节拍、32KB 内核堆，内核源码及 MIT 许可证位于 `Middlewares/Third_Party/FreeRTOS/Source/`。Keil 工程已包含编译所需文件。

| 任务 | 入口 | 职责 |
|---|---|---|
| 网页通信 | `Core/Src/web_control.c` | 解析手动命令，提交物料参数、启动和停止请求，回传状态 |
| 自动搬运 | `Core/Src/robot_control.c` | 处理请求队列，独立运行 `logistics_task.c` 中的搬运状态机 |
| 电机总线 | `Core/Src/motor_bus.c` | 统一发送 USART1 电机指令，保持四轮速度与同步帧的完整顺序 |

`Core/Src/app_rtos.c` 创建三个任务，优先级分别为网页 2、自动 3、电机总线 4。舵机脉宽由 TIM1 硬件输出。网页与自动流程共享的运行状态通过快照读取，电机发送由队列和递归互斥量管理。

网页发送三件物料参数和启动指令，主控校验并接受后自主完成搬运。关闭网页、Wi-Fi 断开、ESP32 重启或串口重新握手时，主控保留当前自动任务和物料计划。手动控制在失联时停车。点击“停止全部运动”或“停止自动任务”会终止自动流程。主控掉电后需重新发送计划并恢复起始位置。

本次升级需要配套烧录 STM32 和 ESP32。新增 `MANUAL_RELEASE`（0x0E）处理手动会话释放，`STOP` 和 `PAUSE` 处理用户主动停止。自动运行期间，主控拒绝手动运动、舵机调整和物料计划修改。

## 当前功能

外置供料转盘的 ESP-NOW 联动代码已加入，使用两块 ESP32-S3，转盘以 A4988 驱动 42 步进电机，沿用 K230 → STM32 → 小车 ESP32 的识别链路。配置、接线、两秒时序分析和启用步骤见 [外置供料转盘说明](ESP32/turntable_controller/README.md)。需要先填写双方 STA MAC、将 A4988 MS1/MS2/MS3 接高以匹配默认 16 细分、确认电机电流及机构参数，并启用 STM32 和小车 ESP32 两个编译开关；默认关闭该功能。尚未烧录或实机验证。

- 网页手动控制底盘、夹爪、云台、料盘与升降点动。
- 网页依次设置三件物料的颜色和形状，发送给 STM32 并等待确认后启动自动任务。
- K230 同时按颜色和形状筛选目标；STM32 验证目标属性后进行视觉对准和抓取。
- 起点装载三件物料，再依次投放到左上、右上、右下区域并返回起点；场地配置为 200 × 200 cm。
- 手动控制包含通信超时停车，自动任务包含位置反馈丢失、升降无位移和运动超时等停止处理。

自动任务使用当前模型支持的组合：

| 形状 | 可选颜色 |
|---|---|
| 六棱柱 | 黑色 |
| 五棱柱 | 红色、绿色、黄色 |
| 圆柱 | 红色、绿色、蓝色、黑色 |
| 长方体、奶娃 | 红色、绿色、黄色、蓝色、黑色；颜色由目标区域的 LAB 信息判断 |

“奶娃”对应模型原标签 `b奶娃`，深蓝色圆柱归入蓝色。正方体、三棱锥、浅蓝色和橙色类别不参与自动任务；模型原有的 24 类输出顺序保持不变。

## 使用

1. 用 Keil 打开主控工程并编译、烧录。
2. 用 Arduino IDE 打开 ESP32 工程；保留同目录所有头文件一起编译、烧录。
3. 将 `K230部署/` 中的 Python 文件和 `.kmodel` 模型复制到 K230 SD 卡根目录。主程序为 `main.py`，模型路径为 `/sdcard/yolo11n_det_320.kmodel`。
4. 连接 ESP32 热点 `Robot-Car`（默认密码 `robot12345`），打开 `http://192.168.4.1/`。申请控制后，设置三件物料，点击“发送抓取顺序”，等待主控确认，再按页面提示启动。

三端程序需配套更新。自动启动前需要恢复起点、车头方向和已标定的搬运高度；当前没有升降自动归零或任务断点续跑。

首次烧录此命名版本后，需要在网页重新保存夹爪、云台的角度预设。

在工程根目录运行以下命令可查看网页效果，预览按钮不会控制实车：

```sh
node "ESP32/网页预览/server.cjs"
```

随后打开 `http://127.0.0.1:8765/`。

## ESP32-S3 N16R8 接线与烧录

N16R8 表示 16MB Flash、8MB PSRAM。使用原来的 `robot_WebControl.ino` 和同目录头文件，网页功能及主控通信协议保持一致；主控已运行配套版本时，不必因更换 S3 再次烧录 STM32。

Arduino IDE 选择以下设置（Arduino ESP32 2.0.3 的菜单名称）：

| 设置 | 选择 |
|---|---|
| 开发板 | ESP32S3 Dev Module |
| Flash Size | 16MB (128Mb) |
| Flash Mode | QIO 80MHz |
| PSRAM | OPI PSRAM |
| Partition Scheme | 16M Flash (3MB APP/9MB FATFS) |
| USB Mode | Hardware CDC and JTAG |
| USB CDC On Boot | Enabled |
| Upload Mode | UART0 / Hardware CDC |
| Upload Speed | 115200 |

| ESP32-S3 | STM32 |
|---|---|
| IO17（TX） | PA1（UART4 RX） |
| IO16（RX） | PA0（UART4 TX） |
| GND | GND |

通信为 115200、8N1、3.3V 电平。接板上 IO16/IO17；标注 TXD/RXD 的引脚通常是 IO43/IO44，与程序设置不同。程序已显式指定串口引脚，保留该设置，避免占用原生 USB 的 IO19/IO20。N16R8 是模块内存规格，如开发板带屏幕等外设，需确认 IO16/IO17 未被占用。

用数据线连接开发板的 USB 下载接口，选择对应端口后上传。若有 USB 和 UART 两个接口，USB 通常连接芯片原生 USB，UART 通常连接 USB 转串口芯片，具体以板上标注为准。上述设置下，`Serial` 调试输出走原生 USB；通过 UART 接口烧录后看不到调试文字不代表程序未运行。首次不能进入下载模式时，按住 BOOT，按一下 RESET/EN，再松开 BOOT，重新选择端口上传。

新板没有旧板保存的夹爪、云台角度预设，烧录后需在网页重新保存。热点仍为 `Robot-Car`，密码 `robot12345`，地址 `http://192.168.4.1/`。更换 S3 不能消除舵机共用电源引起的压降；控制板和舵机建议使用独立降压支路并共地。

## 验证状态

2026-11-05：STM32 Keil 编译通过（0 错误、0 警告）；ESP32 使用 Arduino ESP32 2.0.3 编译通过。网页、API、自动任务模拟及 K230/STM32 双向目标协议测试通过。

2026-11-06：同一份网页程序按上述 ESP32-S3 N16R8 设置编译通过（Arduino ESP32 2.0.3），程序占用 687329 字节，静态内存占用 37972 字节；尚未烧录或进行 S3 实机验证。

2026-11-09，FreeRTOS 版本通过 STM32 Keil 编译，0 错误、0 警告，ROM 占用 46504 字节，静态 RAM 占用 38224 字节（包含 32KB 内核堆）。ESP32-S3 N16R8 配套程序编译通过，程序占用 688161 字节。网页生命周期、串口命令、升降保护、三件自动搬运及 RTOS 请求队列和电机总线并发模拟均通过。RTOS 回归入口为 `tests/rtos_review/build_runtime.cmd`，任务栈余量和实机响应时间需要上板验证。

完整自动搬运尚未完成实车测试，硬件结构及机构参数仍需调整和标定。软件模拟通过不等于实车搬运成功。`tests/` 中部分构建脚本使用本机 Windows/Visual Studio 路径，在其他电脑上需要调整路径；Python YOLO 测试需要 NumPy。

第三方组件保留各目录内的原始许可证。本仓库未另行指定整个工程的统一开源许可证。
