- The Waveshare display has the backlight control pin connected through a CH422G I/O Expander.  This means that we cannot PWM the pin in order to dim the display.  Final design should make sure the display can be dimmed.

https://github.com/chabad-source/esphometimer


https://discord.com/channels/429907082951524364/1355311652458860624/1355321302809837748
"You can just use the time print method directly to provide the text value" << for the seven segment clock renderer


https://github.com/esphome/esphome/blob/dev/esphome/components/mpu6050/mpu6050.cpp
https://github.com/esphome/esphome/blob/dev/esphome/components/i2c/i2c.h
https://www.qstcorp.com/upload/pdf/202301/13-52-25%20QMI8658A%20Datasheet%20Rev%20A.pdf