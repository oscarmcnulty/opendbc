#pragma once

#include "opendbc/safety/safety_declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"


static bool volkswagen_mlb_brake_pedal_switch = false;
static bool volkswagen_mlb_brake_pressure_detected = false;

static safety_config volkswagen_mlb_init(uint16_t param) {
  // Transmit of LS_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MLB_STOCK_TX_MSGS[] = {
    {MSG_HCA_01, 0, 8, .check_relay = true},
    {MSG_LS_01, 0, 4, .check_relay = false},
    {MSG_LS_01, 2, 4, .check_relay = false},
    {MSG_LDW_02, 0, 8, .check_relay = true}
  };

  static const CanMsg VOLKSWAGEN_MLB_LONG_TX_MSGS[] = {
    {MSG_HCA_01, 0, 8, .check_relay = true},
    {MSG_LS_01, 0, 4, .check_relay = false},
    {MSG_LS_01, 2, 4, .check_relay = false},
    {MSG_LDW_02, 0, 8, .check_relay = true},
    {MSG_ACC_02, 0, 8, .check_relay = true},
    {MSG_ACC_01, 0, 8, .check_relay = true},
  };

  static RxCheck volkswagen_mlb_rx_checks[] = {
    {.msg = {{MSG_ESP_03, 0, 8, .ignore_checksum = true, .max_counter = 15U, .frequency = 50U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_LH_EPS_03, 0, 8, .max_counter = 15U, .frequency = 100U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESP_05, 0, 8, .ignore_checksum = true, .max_counter = 15U, .frequency = 50U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_TSK_02, 0, 8, .ignore_checksum = true, .max_counter = 15U, .frequency = 50U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_03, 0, 8, .ignore_checksum = true, .max_counter = 15U, .frequency = 100U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_LS_01, 0, 4, .ignore_checksum = true, .max_counter = 15U, .frequency = 10U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_ACC_02, 2, 8, .ignore_checksum = true, .max_counter = 15U, .frequency = 17U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  UNUSED(param);

  volkswagen_set_button_prev = false;
  volkswagen_resume_button_prev = false;
  volkswagen_mlb_brake_pedal_switch = false;
  volkswagen_mlb_brake_pressure_detected = false;

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#endif
  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);
  return volkswagen_longitudinal ? BUILD_SAFETY_CFG(volkswagen_mlb_rx_checks, VOLKSWAGEN_MLB_LONG_TX_MSGS) : \
                                   BUILD_SAFETY_CFG(volkswagen_mlb_rx_checks, VOLKSWAGEN_MLB_STOCK_TX_MSGS);
}

static void volkswagen_mlb_rx_hook(const CANPacket_t *msg) {

  if (msg->bus == 0U) {
    // Check all wheel speeds for any movement
    // Signals: ESP_03.ESP_[VL|VR|HL|HR]_Radgeschw
    if (msg->addr == MSG_ESP_03) {
      uint32_t speed = 0;
      speed += ((msg->data[3] & 0x0FU) << 8) | msg->data[2];  // FL
      speed += (msg->data[4] << 4) | (msg->data[3] >> 4);    // FR
      speed += ((msg->data[6] & 0x0FU) << 8) | msg->data[5];  // RL
      speed += (msg->data[7] << 4) | (msg->data[6] >> 4);    // RR
      vehicle_moving = speed > 0U;
    }

    // Update driver input torque
    if (msg->addr == MSG_LH_EPS_03) {
      // Signal: LH_EPS_03.EPS_Lenkmoment (absolute torque)
      // Read the 10-bit absolute torque value from bits 40-49
      int torque_driver_new = (msg->data[5]) | ((msg->data[6] & 0x03U) << 8);

      // Signal: LH_EPS_03.EPS_VZ_Lenkmoment (direction)
      // The sign bit is at bit 55
      int sign = (msg->data[6] >> 7) & 1U;

      if (sign == 1) {
        torque_driver_new *= -1;
      }
      update_sample(&torque_driver, torque_driver_new);
    }

    if (msg->addr == MSG_TSK_02) {
      // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
      // Always exit controls on main switch off
      // Signal: TSK_02.TSK_Status
      int acc_status = (msg->data[2] & 0x3U);
      bool cruise_engaged = (acc_status == 1) || (acc_status == 2);
      acc_main_on = cruise_engaged || (acc_status == 0);  // FIXME: this is wrong

      if (!volkswagen_longitudinal) {
        pcm_cruise_check(cruise_engaged);
      }

      // FIXME: cruise main switch state not yet properly detected
      // if (!acc_main_on) {
      //   controls_allowed = false;
      // }
    }

    if (msg->addr == MSG_LS_01) {
      // If using openpilot longitudinal, enter controls on falling edge of Set or Resume with main switch on
      // Signal: LS_01.LS_Tip_Setzen
      // Signal: LS_01.LS_Tip_Wiederaufnahme
      if (volkswagen_longitudinal) {
        bool set_button = GET_BIT(msg, 16U);
        bool resume_button = GET_BIT(msg, 19U);
        if ((volkswagen_set_button_prev && !set_button) ||
            (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = GET_BIT(msg, 12U);  // LS_Hauptschalter
        }
        volkswagen_set_button_prev = set_button;
        volkswagen_resume_button_prev = resume_button;
      }
      // Always exit controls on rising edge of Cancel
      // Signal: LS_01.LS_Abbrechen : 13|1@1+
      if (GET_BIT(msg, 13U) == 1U) {
        controls_allowed = false;
      }
    }

    // Signal: Motor_03.MO_Fahrpedalrohwert_01
    // Signal: Motor_03.MO_Fahrer_bremst
    if (msg->addr == MSG_MOTOR_03) {
      gas_pressed = msg->data[6] != 0U;
      volkswagen_mlb_brake_pedal_switch = GET_BIT(msg, 35U);
    }

    if (msg->addr == MSG_ESP_05) {
      // Signal: ESP_05.ESP_Fahrer_bremst (ESP detected driver brake pressure above threshold)
      volkswagen_mlb_brake_pressure_detected = (msg->data[3] & 0x4U) >> 2;
    }

    brake_pressed = volkswagen_mlb_brake_pedal_switch || volkswagen_mlb_brake_pressure_detected;
  }
}

static bool volkswagen_mlb_tx_hook(const CANPacket_t *msg) {
  // lateral limits
  const TorqueSteeringLimits VOLKSWAGEN_MLB_STEERING_LIMITS = {
    .max_torque = 300,              // 3.0 Nm (EPS side max of 3.0Nm with fault if violated)
    .max_rt_delta = 188,           // 10 max rate up * 50Hz send rate * 250000 RT interval / 1000000 = 125 ; 125 * 1.5 for safety pad = 187.5
    .max_rate_up = 10,             // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
    .max_rate_down = 10,           // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
    .driver_torque_allowance = 80,
    .driver_torque_multiplier = 3,
    .type = TorqueDriverLimited,
  };

  // longitudinal limits
  // acceleration in m/s2 * 1000 to avoid floating point math
  const LongitudinalLimits VOLKSWAGEN_MLB_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 0,
  };

  bool tx = true;

  // Safety check for HCA_01 Heading Control Assist torque
  // Signal: HCA_01.HCA_01_LM_Offset (absolute torque)
  // Signal: HCA_01.HCA_01_LM_OffSign (direction)
  if (msg->addr == MSG_HCA_01) {

    int desired_torque = msg->data[2] | ((msg->data[3] & 0x3FU) << 8);
    int sign = (msg->data[3] & 0x80U) >> 7;
    if (sign == 1) {
      desired_torque *= -1;
    }

    if (steer_torque_cmd_checks(desired_torque, -1, VOLKSWAGEN_MLB_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // Safety check for ACC_01 acceleration request
  // To avoid floating point math, scale upward and compare to pre-scaled safety m/s^2 boundaries
  if (msg->addr == MSG_ACC_01) {
    bool violation = false;
    int desired_accel = 0;

    // Signal: ACC_01.ACC_Sollbeschleunigung (acceleration in m/s^2, scale 0.005, offset -7.22)
    desired_accel = ((((msg->data[4] & 0x07U) << 8) | msg->data[3]) * 5U) - 7220U;

    violation |= longitudinal_accel_checks(desired_accel, VOLKSWAGEN_MLB_LONG_LIMITS);

    if (violation) {
      tx = false;
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((msg->addr == MSG_LS_01) && !controls_allowed) {
    // disallow resume and set: bits 16 and 19
    if ((msg->data[2] & 0x9U) != 0U) {
      tx = false;
    }
  }

  return tx;
}

static bool volkswagen_mlb_fwd_hook(int bus_num, int addr) {
  // Block ACC_01 messages in order to use ACC_05
  if (addr == MSG_ACC_01 && bus_num == 2) {
    return true;
  }
  return false;
}

const safety_hooks volkswagen_mlb_hooks = {
  .init = volkswagen_mlb_init,
  .rx = volkswagen_mlb_rx_hook,
  .tx = volkswagen_mlb_tx_hook,
  .fwd = volkswagen_mlb_fwd_hook,
  .get_counter = volkswagen_mqb_meb_mlb_get_counter,
  .get_checksum = volkswagen_mqb_meb_mlb_get_checksum,
  .compute_checksum = volkswagen_mqb_meb_mlb_compute_crc,
};