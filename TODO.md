- The Waveshare display has the backlight control pin connected through a CH422G I/O Expander.  This means that we cannot PWM the pin in order to dim the display.  Final design should make sure the display can be dimmed.

https://github.com/chabad-source/esphometimer


https://discord.com/channels/429907082951524364/1355311652458860624/1355321302809837748
"You can just use the time print method directly to provide the text value" << for the seven segment clock renderer


https://github.com/esphome/esphome/blob/dev/esphome/components/mpu6050/mpu6050.cpp
https://github.com/esphome/esphome/blob/dev/esphome/components/i2c/i2c.h
https://www.qstcorp.com/upload/pdf/202301/13-52-25%20QMI8658A%20Datasheet%20Rev%20A.pdf

Have the screen auto-return to the clockface after X minutes of inactivity
Build a configuration page:
    - set timezone
    - set screen brightness
    - set nighttime screen brightness
    - set times to use nighttime screen brightness

Look into leveraging code from https://github.com/jdlambert/micro_tz_db to allow for friendly changing of timezones

Check out the Adafruit HUSB238 module as an example of how to get 12V USB-PD and still get the data pins through
Likely just don't need to connect the D+/- pins to the HUSB238
Look into the ESPHome datetime component to simplify the alarms.  Might be able to get rid of the hour/minute numbers.
