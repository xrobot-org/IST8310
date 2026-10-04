# IST8310

iSentek IST8310 三轴磁力计（I2C）驱动模块 / Driver Module for the iSentek IST8310 3-axis magnetometer over I2C

## 1. 模块作用 / Purpose

构造时，IST8310 复位芯片（`rst` 拉低 50 ms 再拉高），通过 I2C（7 位地址 `0x0E`）读取 `WHO_AM_I`（期望 `0x10`），并配置 DRDY 低电平有效、2 次平均、单次测量模式；初始化失败时每 100 ms 重试，直到成功。随后创建线程 `ist8310_thread`（REALTIME 优先级）：每次触发一次单次测量，等待 DRDY 中断（超时 100 ms 则输出警告并重新触发），读取 6 字节原始数据，乘以 0.3（`IST8310_MAG_SEN`，µT/LSB），经 `rotation` 旋转后发布。原始值全为 0 的样本被丢弃，此时重新发布上一次的值（尚无有效样本时为零向量）。

`OnMonitor()` 在数据出现 NaN 或 Inf 时输出警告。

模块在 RamFS 中注册命令 `ist8310`：

- `ist8310`：打印用法。
- `ist8310 show <time_ms> <interval_ms>`：在 `time_ms` 内每隔 `interval_ms` 打印一次磁场。

Upon construction, IST8310 resets the chip (`rst` low for 50 ms, then high), reads `WHO_AM_I` over I2C (7-bit address `0x0E`, expects `0x10`) and configures DRDY active low, 2x averaging and single-measurement mode; on failure it retries every 100 ms until it succeeds. It then creates the thread `ist8310_thread` (REALTIME priority): it triggers one single measurement, waits for the DRDY interrupt (on a 100 ms timeout it logs a warning and triggers again), reads the 6 raw bytes, scales them by 0.3 (`IST8310_MAG_SEN`, µT/LSB), rotates the vector by `rotation` and publishes it. A sample whose raw values are all zero is discarded and the previous value is published again (a zero vector while no valid sample has been received).

`OnMonitor()` logs a warning when the data contains NaN or Inf.

The Module registers the command `ist8310` in RamFS:

- `ist8310`: print the usage.
- `ist8310 show <time_ms> <interval_ms>`: print the magnetic field every `interval_ms` for `time_ms`.

## 2. 构造接口 / Constructor

```cpp
IST8310(LibXR::GPIO& interrupt, LibXR::GPIO& rst, LibXR::I2C& i2c,
        LibXR::RamFS& ramfs,
        LibXR::Quaternion<float>&& rotation = {1.0f, 0.0f, 0.0f, 0.0f},
        const char* topic_name = "ist8310_mag",
        size_t task_stack_depth = 1536);
```

依赖：

- `interrupt`：连接 DRDY 引脚的中断 GPIO，DRDY 低电平有效，由 BSP 配置为下降沿中断，取自 BSP 的硬件注册（`XR_REGISTER`）。
- `rst`：连接复位引脚的输出 GPIO，取自 BSP 的硬件注册。
- `i2c`：芯片所在的 `LibXR::I2C` 总线，取自 BSP 的硬件注册。
- `ramfs`：注册 `ist8310` 命令的 `LibXR::RamFS`，取自 BSP 的硬件注册。

配置参数：

- `rotation`：传感器坐标系到应用坐标系的四元数 `{w, x, y, z}`，默认单位四元数。
- `topic_name`：发布磁场数据的 Topic 名称，默认 `"ist8310_mag"`。
- `task_stack_depth`：采集线程栈深，单位字节，默认 1536。

Dependencies:

- `interrupt`: interrupt GPIO connected to the DRDY pin; DRDY is active low and the BSP configures the GPIO as a falling-edge interrupt; taken from the BSP's Registration (`XR_REGISTER`).
- `rst`: output GPIO connected to the reset pin, taken from the BSP's Registration.
- `i2c`: the `LibXR::I2C` bus of the chip, taken from the BSP's Registration.
- `ramfs`: the `LibXR::RamFS` that receives the `ist8310` command, taken from the BSP's Registration.

Configuration parameters:

- `rotation`: quaternion `{w, x, y, z}` from the sensor frame to the application frame, default identity.
- `topic_name`: name of the published magnetic-field Topic, default `"ist8310_mag"`.
- `task_stack_depth`: stack depth of the acquisition thread in bytes, default 1536.

## 3. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `topic_name`（默认 `ist8310_mag`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 磁场向量 x, y, z，单位 µT，已旋转 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `topic_name` (default `ist8310_mag`) | Publish | `Eigen::Matrix<float, 3, 1>` | Magnetic field vector x, y, z in µT, rotated |

## 4. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/IST8310` 写入的实例，依赖填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/IST8310`, with the dependencies set to names registered by the BSP with `XR_REGISTER` (Registration):

```yaml
modules:
  - module: xrobot-org/IST8310
    id: ist8310_0
    args:
      - interrupt: CMPS_INT
      - rst: CMPS_RST
      - i2c: i2c3
      - ramfs: ramfs
      - rotation: '{1.0f, 0.0f, 0.0f, 0.0f}'
      - topic_name: "ist8310_mag"
      - task_stack_depth: 1536
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一片通过 I2C 连接的 IST8310，DRDY 引脚接一个中断 GPIO，复位引脚接一个输出 GPIO；I2C、GPIO 与 RamFS 由 BSP 通过 `XR_REGISTER` 注册。

Dependencies: LibXR.

Hardware: one IST8310 connected over I2C, with the DRDY pin wired to an interrupt GPIO and the reset pin wired to an output GPIO; the I2C, GPIOs and RamFS are registered by the BSP with `XR_REGISTER`.
