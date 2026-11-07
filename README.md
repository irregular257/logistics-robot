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
| CubeMX 配置 | `robot.ioc` | STM32CubeMX |
| 软件测试 | `tests/` | Python、Node.js、MSVC 主机模拟 |

`Core/` 为主控应用，`HARDWARE/` 为外设驱动，`Drivers/` 为 STM32 HAL/CMSIS。Keil 编译产物和本机缓存不纳入版本管理。

## 当前功能

- 网页手动控制底盘、夹爪、云台、料盘与升降点动。
- 网页依次设置三件物料的颜色和形状，发送给 STM32 并等待确认后启动自动任务。
- K230 同时按颜色和形状筛选目标；STM32 验证目标属性后进行视觉对准和抓取。
- 起点装载三件物料，再依次投放到左上、右上、右下区域并返回起点；场地配置为 200 × 200 cm。
- 包含通信超时、位置反馈丢失、升降无位移和运动超时等停止处理。

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

完整自动搬运尚未完成实车测试，硬件结构及机构参数仍需调整和标定。软件模拟通过不等于实车搬运成功。`tests/` 中部分构建脚本使用本机 Windows/Visual Studio 路径，在其他电脑上需要调整路径；Python YOLO 测试需要 NumPy。

第三方组件保留各目录内的原始许可证。本仓库未另行指定整个工程的统一开源许可证。
