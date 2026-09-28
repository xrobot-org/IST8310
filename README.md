# IST8310

iSentek IST8310 三轴磁力计驱动模块。
Driver module for the iSentek IST8310 3-axis magnetometer.

构造时模块复位芯片（`rst` 拉低 50 ms 再拉高），通过 I2C（7 位地址 `0x0E`）读取
`WHO_AM_I`（期望 `0x10`），配置 DRDY 低有效、2 次平均、单次测量模式；初始化失败时
每 100 ms 重试，直到成功。随后创建 `ist8310_thread` 线程（`REALTIME` 优先级）：
每次触发一次单次测量，等待 DRDY 中断（超时 100 ms 则告警并重新触发），读取 6 字节
原始数据，乘以 0.3（`IST8310_MAG_SEN`，µT/LSB），经 `rotation` 旋转后发布。
原始值全为 0 的样本被丢弃，此时重新发布上一次的值。

During construction the module resets the chip (`rst` low for 50 ms, then high),
reads `WHO_AM_I` over I2C (7-bit address `0x0E`, expects `0x10`) and configures
DRDY active low, 2x averaging and single-measurement mode; on failure it retries
every 100 ms until it succeeds. It then starts the `ist8310_thread` thread
(`REALTIME` priority): it triggers a single measurement, waits for the DRDY
interrupt (on a 100 ms timeout it logs a warning and triggers again), reads the
6 raw bytes, scales them by 0.3 (`IST8310_MAG_SEN`, µT/LSB), rotates the vector
by `rotation` and publishes it. A sample whose raw values are all zero is
discarded and the previous value is published again.

- Topic：`topic_name`（默认 `ist8310_mag`），类型 `Eigen::Matrix<float, 3, 1>`（x, y, z，µT）。
  / Topic `topic_name` (default `ist8310_mag`), type `Eigen::Matrix<float, 3, 1>` (x, y, z in µT).
- `OnMonitor()`：数据出现 NaN 或 Inf 时输出告警。/ Logs a warning when the data contains NaN or Inf.

### RamFS 命令 / RamFS command

模块在 `ramfs` 中注册命令 `ist8310`。/ The module adds the command `ist8310` to `ramfs`.

```sh
ist8310 show <time_ms> <interval_ms>   # 每 interval_ms 打印一次磁场，持续 time_ms / print the field every interval_ms for time_ms
```

## 依赖 / Dependencies

无其他模块依赖，仅使用 LibXR。
No other Modules; LibXR only.

## 构造接口 / Constructor

```cpp
IST8310(LibXR::GPIO& interrupt, LibXR::GPIO& rst, LibXR::I2C& i2c,
        LibXR::RamFS& ramfs,
        LibXR::Quaternion<float>&& rotation = {1.0f, 0.0f, 0.0f, 0.0f},
        const char* topic_name = "ist8310_mag",
        size_t task_stack_depth = 1536);
```

依赖 / Dependencies:

- `interrupt`：连接 DRDY 引脚的中断 GPIO。/ Interrupt GPIO connected to the DRDY pin.
- `rst`：连接复位引脚的输出 GPIO。/ Output GPIO connected to the reset pin.
- `i2c`：芯片所在的 I2C 总线。/ The I2C bus the chip is on.
- `ramfs`：注册 `ist8310` 命令的 RamFS。/ RamFS that receives the `ist8310` command.

配置 / Configuration:

- `rotation`：安装姿态四元数 (w, x, y, z)，默认单位四元数。/ Mounting rotation quaternion (w, x, y, z), identity by default.
- `topic_name`：发布磁场数据的 Topic 名，默认 `ist8310_mag`。/ Name of the magnetic-field topic, default `ist8310_mag`.
- `task_stack_depth`：采集线程栈大小，默认 1536。/ Stack size of the acquisition thread, default 1536.

## 使用 / Use

```sh
xrobot module add xrobot-org/IST8310
xrobot setup
xrobot instance add xrobot-org/IST8310
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把依赖项填为 BSP 中用 `XR_REGISTER` 注册的对象名：
`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set the dependencies to the names of
objects the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/IST8310
    id: ist8310_0
    args:
      - interrupt: ist8310_int
      - rst: ist8310_rst
      - i2c: i2c3
      - ramfs: ramfs
      - rotation: '{1.0f, 0.0f, 0.0f, 0.0f}'
      - topic_name: '"ist8310_mag"'
      - task_stack_depth: '1536'
```

BSP 侧 / BSP side:

```cpp
XR_REGISTER(ist8310_int, LibXR::GPIO);
XR_REGISTER(ist8310_rst, LibXR::GPIO);
XR_REGISTER(i2c3, LibXR::I2C);
XR_REGISTER(ramfs, LibXR::RamFS);
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。
Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/xrobot-org/IST8310`
（在 BSP 中）打印当前的构造函数。
`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/IST8310` in a BSP, prints the current
constructor.
