# Power Meter 4 Lines (ESPHome)

Проект на ESPHome для измерения тока, напряжения и раздельной мощности import/export по четырём линиям 220 В.

- Плата: ESP32 WROOM-32E (`board: esp32dev`)
- АЦП: 2× ADS1115 (один для токов, один для напряжений)
- Датчики: 4× CT 100 A, 4× ZMPT101B
- Логика: кастомный компонент `PowerMeter4Lines` на C++ (`power_meter_ads1115_4lines.h`)

Основной конфиг: `power_meter_4lines.yaml`.

