基于Arduino实现的Modbus通信库，目前暂时用于ESP32。也可以简单移植到STM32以及其他Arduino平台的单片机。
依赖我自己实现的RS485库和CRC16库。
该库实现了Modbus主站和从站的功能，附带Modbus寄存器库。
作为Modbus从站的时候，可以创建Modbus寄存器，或者将外部变量映射到Modbus寄存器空间