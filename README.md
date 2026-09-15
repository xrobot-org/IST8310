# IST8310

## Static assembly source line

This source line uses explicit C++ constructor dependencies and ordered instance
arguments. Inspect the current primary header with `xrobot_mod_parser --path .`;
its declarations, not old manifest/config examples, define the interface.
Historical HardwareContainer/ApplicationManager examples below apply only to the
older dynamic source tags. Device/protocol descriptions remain relevant.
See the XRobot [migration guide](https://github.com/xrobot-org/XRobot/blob/dev/MIGRATION.md).
Compilation is not hardware validation; retain version-specific board evidence.


iSentek IST8310 三轴磁力计驱动模块 / Driver module for iSentek IST8310 3-axis magnetometer

## 硬件需求 / Required Hardware

i2c\_ist8310, ist8310\_int, ist8310\_rst, ramfs

## 构造参数 / Constructor Arguments

* rotation:        {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
* topic\_name:      "ist8310\_mag"
* task\_stack\_depth: 1536

## 依赖 / Depends

无（No dependencies）
