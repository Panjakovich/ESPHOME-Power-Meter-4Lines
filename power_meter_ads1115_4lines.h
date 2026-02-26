#pragma once

#include "esphome.h"
#include "esphome/components/ads1115/ads1115.h"
#include <cmath>

using namespace esphome;

class PowerMeter4Lines : public PollingComponent {
 public:
  PowerMeter4Lines(ads1115::ADS1115Component *ads_current,
                   ads1115::ADS1115Component *ads_voltage,
                   float mains_freq_hz,
                   uint16_t periods_in_window,
                   float current_scale,
                   float voltage_scale)
      : PollingComponent(1000),
        ads_current_(ads_current),
        ads_voltage_(ads_voltage),
        mains_freq_hz_(mains_freq_hz),
        periods_(periods_in_window),
        current_scale_(current_scale),
        voltage_scale_(voltage_scale) {}

  // LINE 1
  Sensor *line1_current = new Sensor();
  Sensor *line1_voltage = new Sensor();
  Sensor *line1_power_import = new Sensor();
  Sensor *line1_power_export = new Sensor();
  Sensor *line1_energy_import = new Sensor();
  Sensor *line1_energy_export = new Sensor();

  // LINE 2
  Sensor *line2_current = new Sensor();
  Sensor *line2_voltage = new Sensor();
  Sensor *line2_power_import = new Sensor();
  Sensor *line2_power_export = new Sensor();
  Sensor *line2_energy_import = new Sensor();
  Sensor *line2_energy_export = new Sensor();

  // LINE 3
  Sensor *line3_current = new Sensor();
  Sensor *line3_voltage = new Sensor();
  Sensor *line3_power_import = new Sensor();
  Sensor *line3_power_export = new Sensor();
  Sensor *line3_energy_import = new Sensor();
  Sensor *line3_energy_export = new Sensor();

  // LINE 4
  Sensor *line4_current = new Sensor();
  Sensor *line4_voltage = new Sensor();
  Sensor *line4_power_import = new Sensor();
  Sensor *line4_power_export = new Sensor();
  Sensor *line4_energy_import = new Sensor();
  Sensor *line4_energy_export = new Sensor();

  void setup() override {
    if (this->ads_current_ != nullptr) {
      this->ads_current_->set_continuous_mode(true);
    }
    if (this->ads_voltage_ != nullptr) {
      this->ads_voltage_->set_continuous_mode(true);
    }
  }

  void update() override {
    if (this->ads_current_ == nullptr || this->ads_voltage_ == nullptr) {
      ESP_LOGW("power_meter_4lines", "ADS1115 not initialized");
      return;
    }

    const uint16_t samples = 32 * this->periods_;

    float sum_u[4]   = {0, 0, 0, 0};
    float sum_i[4]   = {0, 0, 0, 0};
    float sum_u2[4]  = {0, 0, 0, 0};
    float sum_i2[4]  = {0, 0, 0, 0};
    float sum_ui[4]  = {0, 0, 0, 0};

    using namespace esphome::ads1115;

    for (uint16_t n = 0; n < samples; n++) {
      float u_adc[4];
      u_adc[0] = this->ads_voltage_->request_measurement(
          ADS1115_MULTIPLEXER_P0_NG,
          ADS1115_GAIN_2P048,
          ADS1115_16_BITS,
          ADS1115_860SPS);

      u_adc[1] = this->ads_voltage_->request_measurement(
          ADS1115_MULTIPLEXER_P1_NG,
          ADS1115_GAIN_2P048,
          ADS1115_16_BITS,
          ADS1115_860SPS);

      u_adc[2] = this->ads_voltage_->request_measurement(
          ADS1115_MULTIPLEXER_P2_NG,
          ADS1115_GAIN_2P048,
          ADS1115_16_BITS,
          ADS1115_860SPS);

      u_adc[3] = this->ads_voltage_->request_measurement(
          ADS1115_MULTIPLEXER_P3_NG,
          ADS1115_GAIN_2P048,
          ADS1115_16_BITS,
          ADS1115_860SPS);

      float i_adc[4];
      i_adc[0] = this->ads_current_->request_measurement(
          ADS1115_MULTIPLEXER_P0_NG,
          ADS1115_GAIN_2P048,
          ADS1115_16_BITS,
          ADS1115_860SPS);

      i_adc[1] = this->ads_current_->request_measurement(
          ADS1115_MULTIPLEXER_P1_NG,
          ADS1115_GAIN_2P048,
          ADS1115_16_BITS,
          ADS1115_860SPS);

      i_adc[2] = this->ads_current_->request_measurement(
          ADS1115_MULTIPLEXER_P2_NG,
          ADS1115_GAIN_2P048,
          ADS1115_16_BITS,
          ADS1115_860SPS);

      i_adc[3] = this->ads_current_->request_measurement(
          ADS1115_MULTIPLEXER_P3_NG,
          ADS1115_GAIN_2P048,
          ADS1115_16_BITS,
          ADS1115_860SPS);

      for (int ch = 0; ch < 4; ch++) {
        float u = u_adc[ch];
        float i = i_adc[ch];

        sum_u[ch]  += u;
        sum_i[ch]  += i;
        sum_u2[ch] += u * u;
        sum_i2[ch] += i * i;
        sum_ui[ch] += u * i;
      }
    }

    const float N = static_cast<float>(samples);
    float dt_hours = (this->get_update_interval() / 1000.0f) / 3600.0f;

    for (int ch = 0; ch < 4; ch++) {
      float mean_u = sum_u[ch] / N;
      float mean_i = sum_i[ch] / N;

      float var_u = (sum_u2[ch] / N) - mean_u * mean_u;
      float var_i = (sum_i2[ch] / N) - mean_i * mean_i;
      float cov_ui = (sum_ui[ch] / N) - mean_u * mean_i;

      if (var_u < 0) var_u = 0;
      if (var_i < 0) var_i = 0;

      float u_rms_adc = std::sqrt(var_u);
      float i_rms_adc = std::sqrt(var_i);

      float u_rms = u_rms_adc * this->voltage_scale_;
      float i_rms = i_rms_adc * this->current_scale_;

      float p_watts = cov_ui * this->voltage_scale_ * this->current_scale_;

      float p_import = (p_watts > 0.0f) ? p_watts : 0.0f;
      float p_export = (p_watts < 0.0f) ? -p_watts : 0.0f;

      energy_import_kwh_[ch] += p_import * dt_hours / 1000.0f;
      energy_export_kwh_[ch] += p_export * dt_hours / 1000.0f;

      switch (ch) {
        case 0:
          this->line1_voltage->publish_state(u_rms);
          this->line1_current->publish_state(i_rms);
          this->line1_power_import->publish_state(p_import);
          this->line1_power_export->publish_state(p_export);
          this->line1_energy_import->publish_state(energy_import_kwh_[ch]);
          this->line1_energy_export->publish_state(energy_export_kwh_[ch]);
          break;
        case 1:
          this->line2_voltage->publish_state(u_rms);
          this->line2_current->publish_state(i_rms);
          this->line2_power_import->publish_state(p_import);
          this->line2_power_export->publish_state(p_export);
          this->line2_energy_import->publish_state(energy_import_kwh_[ch]);
          this->line2_energy_export->publish_state(energy_export_kwh_[ch]);
          break;
        case 2:
          this->line3_voltage->publish_state(u_rms);
          this->line3_current->publish_state(i_rms);
          this->line3_power_import->publish_state(p_import);
          this->line3_power_export->publish_state(p_export);
          this->line3_energy_import->publish_state(energy_import_kwh_[ch]);
          this->line3_energy_export->publish_state(energy_export_kwh_[ch]);
          break;
        case 3:
          this->line4_voltage->publish_state(u_rms);
          this->line4_current->publish_state(i_rms);
          this->line4_power_import->publish_state(p_import);
          this->line4_power_export->publish_state(p_export);
          this->line4_energy_import->publish_state(energy_import_kwh_[ch]);
          this->line4_energy_export->publish_state(energy_export_kwh_[ch]);
          break;
      }
    }
  }

 protected:
  ads1115::ADS1115Component *ads_current_{nullptr};
  ads1115::ADS1115Component *ads_voltage_{nullptr};

  float mains_freq_hz_{50.0f};
  uint16_t periods_{20};
  float current_scale_{1.0f};
  float voltage_scale_{1.0f};

  float energy_import_kwh_[4] = {0, 0, 0, 0};
  float energy_export_kwh_[4] = {0, 0, 0, 0};
};

