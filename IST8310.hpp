#pragma once

// clang-format off
/* === MODULE MANIFEST ===
module_name: IST8310
module_description: iSentek IST8310 三轴磁力计驱动模块 / Driver module for iSentek IST8310 3-axis magnetometer
constructor_args:
  - rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
  - topic_name: "ist8310_mag"
  - task_stack_depth: 384
required_hardware: i2c_ist8310 ist8310_int ist8310_rst ramfs
repository: https://github.com/xrobot-org/IST8310
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "message.hpp"
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

#define IST8310_MAG_SEN (0.3f)

#define IST8310_WHO_AM_I_RESPONSE (0x10)

#define IST8310_I2C_ADDR (0x0E << 1)  // Default 7-bit address

#define IST8310_MAG_RX_LEN (6)

class IST8310 : public LibXR::Application {
 public:
  IST8310(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
          LibXR::Quaternion<float> &&rotation, const char *topic_name,
          size_t task_stack_depth)
      : rotation_(std::move(rotation)),
        topic_mag_(topic_name, sizeof(mag_data_)),
        int_drdy_(hw.template FindOrExit<LibXR::GPIO>({"ist8310_int"})),
        reset_(hw.template FindOrExit<LibXR::GPIO>({"ist8310_rst"})),
        i2c_(hw.template FindOrExit<LibXR::I2C>({"i2c_ist8310"})),
        op_i2c_read_(sem_i2c_),
        op_i2c_write_(sem_i2c_),
        cmd_file_(LibXR::RamFS::CreateFile("ist8310", CommandFunc, this)) {
    app.Register(*this);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    int_drdy_->DisableInterrupt();
    auto int_cb = LibXR::GPIO::Callback::Create(
        [](bool in_isr, IST8310 *sensor) {
          sensor->new_data_.PostFromCallback(in_isr);
        },
        this);
    int_drdy_->RegisterCallback(int_cb);

    while (!Init()) {
      XR_LOG_ERROR("IST8310: Init failed, retrying...\r\n");
      LibXR::Thread::Sleep(100);
    }

    XR_LOG_PASS("IST8310: Init succeeded.\r\n");
    thread_.Create(this, ThreadFunc, "ist8310_thread", task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);
  }

  bool Init() {
    reset_->Write(false);
    LibXR::Thread::Sleep(50);
    reset_->Write(true);
    LibXR::Thread::Sleep(50);

    if (ReadReg(IST8310_REG_WHO_AM_I) != IST8310_WHO_AM_I_RESPONSE) {
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

  static void ThreadFunc(IST8310 *sensor) {
    sensor->TriggerMeasurement();
    while (true) {
      if (sensor->new_data_.Wait(100) == ErrorCode::OK) {
        sensor->ReadMagnetometer();
        sensor->ParseMagData();
        sensor->topic_mag_.Publish(sensor->mag_data_);
        sensor->TriggerMeasurement();
      } else {
        sensor->TriggerMeasurement();
        XR_LOG_WARN("IST8310: Measurement timed out.\r\n");
      }
    }
  }

  void TriggerMeasurement() {
    WriteReg(IST8310_REG_CNTL1, 0x01);  // Single measurement mode
  }

  void ReadMagnetometer() {
    i2c_->MemRead(IST8310_I2C_ADDR, IST8310_REG_DATAXL, read_buffer_,
                  op_i2c_read_);
  }

  void ParseMagData() {
    std::array<int16_t, 3> raw;
    for (int i = 0; i < 3; ++i) {
      raw[i] = static_cast<int16_t>((read_buffer_[i * 2 + 1] << 8) |
                                    read_buffer_[i * 2]);
    }

    if (raw[0] == 0 && raw[1] == 0 && raw[2] == 0) {
      return;
    }

    Eigen::Matrix<float, 3, 1> vec;
    for (int i = 0; i < 3; ++i) {
      vec[i] = static_cast<float>(raw[i]) * IST8310_MAG_SEN;
    }
    mag_data_ = rotation_ * vec;
  }

  uint8_t ReadReg(uint8_t reg) {
    uint8_t data = 0;
    i2c_->MemRead(IST8310_I2C_ADDR, reg, data, op_i2c_read_);
    return data;
  }

  void WriteReg(uint8_t reg, uint8_t val) {
    i2c_->MemWrite(IST8310_I2C_ADDR, reg, val, op_i2c_write_);
  }

  void OnMonitor(void) override {
    if (std::isnan(mag_data_.x()) || std::isnan(mag_data_.y()) ||
        std::isnan(mag_data_.z()) || std::isinf(mag_data_.x()) ||
        std::isinf(mag_data_.y()) || std::isinf(mag_data_.z())) {
      XR_LOG_WARN("IST8310: NaN or Inf detected.\r\n");
    }
  }

  static int CommandFunc(IST8310 *sensor, int argc, char **argv) {
    if (argc == 1) {
      LibXR::STDIO::Printf("Usage:\r\n");
      LibXR::STDIO::Printf(
          "  show [time_ms] [interval_ms] - Print sensor data "
          "periodically.\r\n");
    } else if (argc == 4) {
      if (strcmp(argv[1], "show") == 0) {
        int time_ms = atoi(argv[2]);
        int interval_ms = atoi(argv[3]);
        for (int i = 0; i < time_ms / interval_ms; i++) {
          LibXR::Thread::Sleep(interval_ms);
          LibXR::STDIO::Printf("Mag: x=%f, y=%f, z=%f\r\n",
                               sensor->mag_data_.x(), sensor->mag_data_.y(),
                               sensor->mag_data_.z());
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
  LibXR::I2C *i2c_;

  LibXR::Semaphore sem_i2c_, new_data_;
  LibXR::ReadOperation op_i2c_read_;
  LibXR::WriteOperation op_i2c_write_;

  uint8_t read_buffer_[IST8310_MAG_RX_LEN] = {};

  LibXR::RamFS::File cmd_file_;
  LibXR::Thread thread_;
};
