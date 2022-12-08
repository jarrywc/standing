# standing-desk-esp32-rtos

This project is a redesign of a standing desk control board that extends from a button control hardware implementation to extend to a software defined control via mqtt messages relayed from homebridge (NodeJS server).

Dependancies:

- Adafruit_PCF8575 [https://github.com/adafruit/Adafruit_PCF8574]
- jm_LiquidCrystal_I2C [https://github.com/jmparatte/jm_LiquidCrystal_I2C]
- EspMQTTClient [https://github.com/plapointe6/EspMQTTClient]
- FreeRTOS 

I2C
- 1x PCF8575 to four momentary push buttons
- 2x20 LCD

GPIO
- 5x Relays for bipolar motor control and primary power switch

