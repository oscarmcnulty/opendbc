#!/usr/bin/env python3
import math
import unittest
import numpy as np
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags

MAX_ACCEL = 2.0
MIN_ACCEL = -2.95

MSG_ACC_01 = 0x109      # TX by OP, longitudinal drivetrain control
MSG_ACC_02 = 0x30C      # TX by OP, ACC HUD data to the instrument cluster
MSG_ACC_05 = 0x10D      # TX by OP (long), ACC control instructions to the drivetrain coordinator
MSG_ACC_10 = 0x117      # TX by OP (long), ANB low-speed deceleration request to the ESP
MSG_LH_EPS_03 = 0x9F    # RX from EPS, for driver steering torque
MSG_ESP_03 = 0x103      # RX from ABS, for wheel speeds
MSG_MOTOR_03 = 0x105    # RX from ECU, for driver throttle input and driver brake input
MSG_ESP_05 = 0x106      # RX from ABS, for brake light state
MSG_LS_01 = 0x10B       # TX by OP, ACC control buttons for cancel/resume
MSG_TSK_04 = 0x10E      # RX from ECU, for ACC status from drivetrain coordinator
MSG_HCA_01 = 0x126      # TX by OP, Heading Control Assist steering torque
MSG_LDW_02 = 0x397      # TX by OP, Lane line recognition and text alerts


class TestVolkswagenMlbSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_01, MSG_LDW_02)}

  MAX_RATE_UP = 9
  MAX_RATE_DOWN = 10
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 169

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 3

  # Wheel speeds _esp_03_msg
  def _speed_msg(self, speed):
    values = {"ESP_%s_Radgeschw" % s: speed for s in ["HL", "HR", "VL", "VR"]}
    return self.packer.make_can_msg_safety("ESP_03", 0, values)

  # Driver brake pressure over threshold
  def _esp_05_msg(self, brake):
    values = {"ESP_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("ESP_05", 0, values)

  # Brake pedal switch
  def _motor_03_msg(self, brake_signal=False, gas_signal=0):
    values = {
      "MO_BLS": brake_signal,
      "MO_Fahrpedalrohwert_01": gas_signal,
    }
    return self.packer.make_can_msg_safety("Motor_03", 0, values)

  def _user_brake_msg(self, brake):
    return self._motor_03_msg(brake_signal=brake)

  def _user_gas_msg(self, gas):
    return self._motor_03_msg(gas_signal=gas)

  # ACC engagement status
  def _tsk_status_msg(self, enable):
    values = {"TSK_Status_GRA_ACC_02": 1 if enable else 0}
    return self.packer.make_can_msg_safety("TSK_04", 1, values)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  # Driver steering input torque
  def _torque_driver_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": torque < 0}
    return self.packer.make_can_msg_safety("LH_EPS_03", 0, values)

  # openpilot steering output torque
  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"HCA_01_LM_Offset": abs(torque),
              "HCA_01_LM_OffSign": torque < 0,
              "HCA_01_Sendestatus": steer_req,
              "HCA_01_Status_HCA": 7 if steer_req else 3}
    return self.packer.make_can_msg_safety("HCA_01", 0, values)

  # Cruise control buttons
  def _ls_01_msg(self, cancel=0, resume=0, _set=0, main_switch=1, bus=2):
    values = {"LS_Abbrechen": cancel, "LS_Tip_Setzen": _set, "LS_Tip_Wiederaufnahme": resume,
              "LS_Hauptschalter": main_switch}
    return self.packer.make_can_msg_safety("LS_01", bus, values)

  # Verify brake_pressed is true if either the switch or pressure threshold signals are true
  def test_redundant_brake_signals(self):
    test_combinations = [(True, True, True), (True, True, False), (True, False, True), (False, False, False)]
    for brake_pressed, motor_03_signal, esp_05_signal in test_combinations:
      self._rx(self._motor_03_msg(brake_signal=False))
      self._rx(self._esp_05_msg(False))
      self.assertFalse(self.safety.get_brake_pressed_prev())
      self._rx(self._motor_03_msg(brake_signal=motor_03_signal))
      self._rx(self._esp_05_msg(esp_05_signal))
      self.assertEqual(brake_pressed, self.safety.get_brake_pressed_prev(),
                       f"expected {brake_pressed=} with {motor_03_signal=} and {esp_05_signal=}")

  def test_rx_hook_xor_checksum(self):
    # Non-LH_EPS_03 messages use XOR checksum; verify valid messages pass and corrupted ones fail
    msg = self._speed_msg(0)
    self.assertTrue(self._rx(msg))
    msg[0].data[0] ^= 0xFF  # corrupt checksum byte
    self.assertFalse(self._rx(msg))

  def test_torque_measurements(self):
    # TODO: make this test work with all cars
    self._rx(self._torque_driver_msg(50))
    self._rx(self._torque_driver_msg(-50))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))

    self.assertEqual(-50, self.safety.get_torque_driver_min())
    self.assertEqual(50, self.safety.get_torque_driver_max())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(-50, self.safety.get_torque_driver_min())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(0, self.safety.get_torque_driver_min())


class TestVolkswagenMlbStockSafety(TestVolkswagenMlbSafetyBase):
  TX_MSGS = [[MSG_HCA_01, 0], [MSG_LDW_02, 0], [MSG_LS_01, 0], [MSG_LS_01, 2]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_01, MSG_LDW_02]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def setUp(self):
    self.packer = CANPackerSafety("vw_mlb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMlb, 0)
    self.safety.init_tests()

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._ls_01_msg(cancel=1)))
    self.assertFalse(self._tx(self._ls_01_msg(resume=1)))
    self.assertFalse(self._tx(self._ls_01_msg(_set=1)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._ls_01_msg(resume=1)))

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self._rx(self._tsk_status_msg(False))
    self.safety.set_controls_allowed(1)
    self._rx(self._ls_01_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")


class TestVolkswagenMlbLongSafety(TestVolkswagenMlbSafetyBase):
  TX_MSGS = [[MSG_HCA_01, 0], [MSG_LS_01, 0], [MSG_LS_01, 2], [MSG_LDW_02, 0],
             [MSG_ACC_02, 0], [MSG_ACC_01, 0], [MSG_ACC_05, 0], [MSG_ACC_10, 0]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_01, MSG_LDW_02, MSG_ACC_02, MSG_ACC_01, MSG_ACC_05, MSG_ACC_10]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_01, MSG_LDW_02, MSG_ACC_02, MSG_ACC_01, MSG_ACC_05, MSG_ACC_10)}

  def setUp(self):
    self.packer = CANPackerSafety("vw_mlb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMlb, VolkswagenSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  # Acceleration request to drivetrain coordinator
  def _acc_01_msg(self, accel):
    values = {"ACC_Sollbeschleunigung": accel}
    return self.packer.make_can_msg_safety("ACC_01", 0, values)

  # ANB low-speed deceleration request and AWV standstill hold request to the ESP
  def _acc_10_msg(self, accel, awv_halten=0):
    values = {"ANB_Zielbrems_Teilbrems_Verz_Anf": accel, "AWV_Halten": awv_halten}
    return self.packer.make_can_msg_safety("ACC_10", 0, values)

  # stock cruise controls are entirely bypassed under openpilot longitudinal control
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_and_resume_buttons(self):
    for button in ["set", "resume"]:
      # ACC main switch must be on, engage on falling edge of button
      self.safety.set_controls_allowed(0)
      self._rx(self._ls_01_msg(_set=(button == "set"), resume=(button == "resume"), main_switch=0, bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} with main switch off")
      self._rx(self._ls_01_msg(_set=(button == "set"), resume=(button == "resume"), main_switch=1, bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} rising edge")
      self._rx(self._ls_01_msg(main_switch=1, bus=0))
      self.assertTrue(self.safety.get_controls_allowed(), f"controls not allowed on {button} falling edge")

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self.safety.set_controls_allowed(1)
    self._rx(self._ls_01_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")

  def test_accel_safety_check(self):
    for controls_allowed in [True, False]:
      # enforce we don't skip over 0 (inactive accel) or exact boundary conditions
      for accel in np.concatenate((np.arange(MIN_ACCEL - 2, MAX_ACCEL + 2, 0.03), [0])):
        accel = round(accel, 2)
        is_inactive_accel = accel == 0
        send = (controls_allowed and MIN_ACCEL <= accel <= MAX_ACCEL) or is_inactive_accel
        self.safety.set_controls_allowed(controls_allowed)
        self.assertEqual(send, self._tx(self._acc_01_msg(accel)), (controls_allowed, accel))

  def test_acc_10_accel_safety_check(self):
    # ANB deceleration request is bounded by the same longitudinal limits as ACC_01. The signal is
    # coarsely quantized (scale 0.024, offset -20.016), so compute the expected result from the value
    # panda actually decodes after the packer rounds, mirroring the safety's integer math.
    for controls_allowed in [True, False]:
      self.safety.set_controls_allowed(controls_allowed)
      for accel in np.concatenate((np.arange(MIN_ACCEL - 2, MAX_ACCEL + 2, 0.024), [0])):
        accel = round(accel, 3)
        raw = math.floor((accel + 20.016) / 0.024 + 0.5)  # packer encoding (round-half-up)
        decoded = (raw * 24 - 20016) / 1000.0  # m/s^2, matches panda's integer decode
        is_inactive_accel = decoded == 0
        send = (controls_allowed and MIN_ACCEL <= decoded <= MAX_ACCEL) or is_inactive_accel
        self.assertEqual(send, self._tx(self._acc_10_msg(accel)), (controls_allowed, accel, decoded))

  def test_acc_10_standstill_hold_safety_check(self):
    # AWV_Halten actuates the brakes to hold at standstill, so it may only be sent while engaged
    for controls_allowed in [True, False]:
      self.safety.set_controls_allowed(controls_allowed)
      self.assertEqual(controls_allowed, self._tx(self._acc_10_msg(0, awv_halten=1)), controls_allowed)
      self.assertTrue(self._tx(self._acc_10_msg(0, awv_halten=0)), controls_allowed)


if __name__ == "__main__":
  unittest.main()
