#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: iSentek IST8310 三轴磁力计（I2C）驱动模块 / Driver Module for the iSentek IST8310 3-axis magnetometer over I2C
depends: []
=== END MANIFEST === */
// clang-format on

#include <memory>

#include "gpio.hpp"
#include "i2c.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "transform.hpp"

#define IST8310_REG_WHO_AM_I (0x00)
#define IST8310_REG_STAT1 (0x02)
#define IST8310_REG_DATAXL (0x03)
#define IST8310_REG_DATAXH (0x04)
#define IST8310_REG_DATAYL (0x05)
#define IST8310_REG_DATAYH (0x06)
#define IST8310_REG_DATAZL (0x07)
#define IST8310_REG_DATAZH (0x08)
#define IST8310_REG_STAT2 (0x09)
#define IST8310_REG_CNTL1 (0x0A)
#define IST8310_REG_CNTL2 (0x0B)
#define IST8310_REG_SELFTEST (0x0C)
#define IST8310_REG_TEMP_LSB (0x1C)
#define IST8310_REG_TEMP_MSB (0x1D)
#define IST8310_REG_AVGCNTL (0x41)
#define IST8310_REG_PDCNTL (0x42)

/// 磁场灵敏度 (µT/LSB)
/// Magnetic sensitivity (µT/LSB)
#define IST8310_MAG_SEN (0.3f)

#define IST8310_WHO_AM_I_RESPONSE (0x10)

/// 默认 7 位 I2C 地址，不含读写位
/// Default 7-bit I2C address without the R/W bit
#define IST8310_I2C_ADDR (0x0E)

#define IST8310_MAG_RX_LEN (6)

/**
 * @brief IST8310 三轴磁力计驱动模块，采集磁场并发布到 Topic。
 *        Driver Module for the IST8310 3-axis magnetometer; it samples the magnetic
 *        field and publishes it to a Topic.
 */
class IST8310
{
 public:
  /**
   * @brief 构造 IST8310：注册 DRDY 中断与 RamFS 命令，初始化芯片，创建采集线程。
   *        Construct IST8310: register the DRDY interrupt and the RamFS command,
   *        initialize the chip, and create the acquisition thread.
   *
   * @param interrupt 连接 DRDY 引脚的中断 GPIO。
   *                  Interrupt GPIO connected to the DRDY pin.
   * @param rst 连接复位引脚的输出 GPIO。
   *            Output GPIO connected to the reset pin.
   * @param i2c 芯片所在的 I2C。
   *            I2C bus of the chip.
   * @param ramfs 接收 `ist8310` 命令的 RamFS。
   *              RamFS that receives the `ist8310` command.
   * @param rotation 传感器坐标系到应用坐标系的四元数 (w, x, y, z)。
   *                 Quaternion (w, x, y, z) from the sensor frame to the application
   *                 frame.
   * @param topic_name 发布磁场数据的 Topic 名称。
   *                   Name of the Topic that publishes the magnetic field.
   * @param task_stack_depth 采集线程栈深。
   *                         Stack depth of the acquisition thread.
   */
  IST8310(
      LibXR::GPIO& interrupt,
      LibXR::GPIO& rst,
      LibXR::I2C& i2c,
      LibXR::RamFS& ramfs,
      LibXR::Quaternion<float>&& rotation = {1.0f, 0.0f, 0.0f, 0.0f},
      const char* topic_name = "ist8310_mag",
      size_t task_stack_depth = 1536)
      : rotation_(std::move(rotation)),
        topic_mag_(LibXR::Topic::CreateTopic<decltype(mag_data_)>(topic_name)),
        int_drdy_(std::addressof(interrupt)),
        reset_(std::addressof(rst)),
        i2c_(std::addressof(i2c)),
        op_i2c_read_(sem_i2c_),
        op_i2c_write_(sem_i2c_),
        cmd_file_(LibXR::RamFS::CreateFile("ist8310", CommandFunc, this))
  {
    ramfs.Add(cmd_file_);

    int_drdy_->DisableInterrupt();
    auto int_cb = LibXR::GPIO::Callback::Create(
        [](bool in_isr, IST8310* sensor) { sensor->new_data_.PostFromCallback(in_isr); },
        this);
    int_drdy_->RegisterCallback(int_cb);

    while (!Init())
    {
      XR_LOG_ERROR("IST8310: Init failed, retrying...\r\n");
      LibXR::Thread::Sleep(100);
    }

    XR_LOG_PASS("IST8310: Init succeeded.\r\n");
    thread_.Create(this, ThreadFunc, "ist8310_thread", task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);
  }

  /**
   * @brief 复位芯片，校验 WHO_AM_I，并配置 DRDY、平均次数与单次测量模式。
   *        Reset the chip, check WHO_AM_I, and configure DRDY, the averaging and the
   *        single-measurement mode.
   *
   * @return 初始化成功返回 true，WHO_AM_I 不符时返回 false。
   *         True on success, false when WHO_AM_I does not match.
   */
  bool Init()
  {
    reset_->Write(false);
    LibXR::Thread::Sleep(50);
    reset_->Write(true);
    LibXR::Thread::Sleep(50);

    if (ReadReg(IST8310_REG_WHO_AM_I) != IST8310_WHO_AM_I_RESPONSE)
    {
      return false;
    }

    WriteReg(IST8310_REG_CNTL2, 0x08);    // DRDY enabled, active low
    WriteReg(IST8310_REG_AVGCNTL, 0x09);  // Avg 2 times
    WriteReg(IST8310_REG_PDCNTL, 0xC0);   // Normal pulse duration
    WriteReg(IST8310_REG_CNTL1, 0x01);    // Single measurement mode

    LibXR::Thread::Sleep(10);

    int_drdy_->EnableInterrupt();
    return true;
  }

  /**
   * @brief 采集线程：触发单次测量，等待 DRDY 中断，读取并发布磁场。
   *        Acquisition thread: trigger a single measurement, wait for the DRDY
   *        interrupt, then read and publish the magnetic field.
   *
   * @param sensor IST8310 实例。
   *               IST8310 instance.
   */
  static void ThreadFunc(IST8310* sensor)
  {
    sensor->TriggerMeasurement();
    while (true)
    {
      if (sensor->new_data_.Wait(100) == LibXR::ErrorCode::OK)
      {
        sensor->ReadMagnetometer();
        sensor->ParseMagData();
        sensor->topic_mag_.Publish(sensor->mag_data_);
        sensor->TriggerMeasurement();
      }
      else
      {
        sensor->TriggerMeasurement();
        XR_LOG_WARN("IST8310: Measurement timed out.\r\n");
      }
    }
  }

  /**
   * @brief 触发一次单次测量。
   *        Trigger one single measurement.
   */
  void TriggerMeasurement()
  {
    WriteReg(IST8310_REG_CNTL1, 0x01);  // Single measurement mode
  }

  /**
   * @brief 读取 6 字节原始磁场数据到内部缓冲区。
   *        Read the 6 raw magnetic-field bytes into the internal buffer.
   */
  void ReadMagnetometer()
  {
    i2c_->MemRead(IST8310_I2C_ADDR, IST8310_REG_DATAXL, read_buffer_, op_i2c_read_);
  }

  /**
   * @brief 解析缓冲区中的原始数据，换算为 µT 并乘以 rotation；原始值全为 0 时不更新。
   *        Parse the raw data in the buffer, convert it to µT and multiply by rotation;
   *        the output is not updated when all raw values are 0.
   */
  void ParseMagData()
  {
    std::array<int16_t, 3> raw;
    for (int i = 0; i < 3; ++i)
    {
      raw[i] = static_cast<int16_t>((read_buffer_[i * 2 + 1] << 8) | read_buffer_[i * 2]);
    }

    if (raw[0] == 0 && raw[1] == 0 && raw[2] == 0)
    {
      return;
    }

    Eigen::Matrix<float, 3, 1> vec;
    for (int i = 0; i < 3; ++i)
    {
      vec[i] = static_cast<float>(raw[i]) * IST8310_MAG_SEN;
    }
    mag_data_ = rotation_ * vec;
  }

  /**
   * @brief 读一个寄存器。
   *        Read one register.
   *
   * @param reg 寄存器地址。
   *            Register address.
   * @return 寄存器的值。
   *         Register value.
   */
  uint8_t ReadReg(uint8_t reg)
  {
    uint8_t data = 0;
    i2c_->MemRead(IST8310_I2C_ADDR, reg, data, op_i2c_read_);
    return data;
  }

  /**
   * @brief 写一个寄存器。
   *        Write one register.
   *
   * @param reg 寄存器地址。
   *            Register address.
   * @param val 写入的值。
   *            Value to write.
   */
  void WriteReg(uint8_t reg, uint8_t val)
  {
    i2c_->MemWrite(IST8310_I2C_ADDR, reg, val, op_i2c_write_);
  }

  /**
   * @brief 监控回调：数据含 NaN 或 Inf 时输出警告。
   *        Monitor callback: log a warning when the data contains NaN or Inf.
   */
  void OnMonitor(void)
  {
    if (std::isnan(mag_data_.x()) || std::isnan(mag_data_.y()) ||
        std::isnan(mag_data_.z()) || std::isinf(mag_data_.x()) ||
        std::isinf(mag_data_.y()) || std::isinf(mag_data_.z()))
    {
      XR_LOG_WARN("IST8310: NaN or Inf detected.\r\n");
    }
  }

  /**
   * @brief RamFS 命令 `ist8310`：周期打印磁场数据。
   *        RamFS command `ist8310`: print the magnetic field periodically.
   *
   * @param sensor IST8310 实例。
   *               IST8310 instance.
   * @param argc 参数个数。
   *             Argument count.
   * @param argv 参数列表。
   *             Argument list.
   * @return 命令处理完成后返回 0。
   *         0 after the command is processed.
   */
  static int CommandFunc(IST8310* sensor, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<
          "  show [time_ms] [interval_ms] - Print sensor data "
          "periodically.\r\n">();
    }
    else if (argc == 4)
    {
      if (strcmp(argv[1], "show") == 0)
      {
        int time_ms = atoi(argv[2]);
        int interval_ms = atoi(argv[3]);
        for (int i = 0; i < time_ms / interval_ms; i++)
        {
          LibXR::Thread::Sleep(interval_ms);
          LibXR::STDIO::Printf<"Mag: x=%f, y=%f, z=%f\r\n">(
              sensor->mag_data_.x(), sensor->mag_data_.y(), sensor->mag_data_.z());
        }
      }
    }

    return 0;
  }

 private:
  LibXR::Quaternion<float> rotation_;
  Eigen::Matrix<float, 3, 1> mag_data_;
  LibXR::Topic topic_mag_;

  LibXR::GPIO *int_drdy_, *reset_;
  LibXR::I2C* i2c_;

  LibXR::Semaphore sem_i2c_, new_data_;
  LibXR::ReadOperation op_i2c_read_;
  LibXR::WriteOperation op_i2c_write_;

  uint8_t read_buffer_[IST8310_MAG_RX_LEN] = {};

  LibXR::RamFS::File cmd_file_;
  LibXR::Thread thread_;
};
