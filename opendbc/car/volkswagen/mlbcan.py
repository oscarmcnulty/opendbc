from opendbc.car.volkswagen.mqbcan import crc8h2f_checksum,xor_checksum


def create_steering_control(packer, bus, apply_steer, lkas_enabled):
  values = {
    "HCA_01_Status_HCA": 7 if lkas_enabled else 3,
    "HCA_01_LM_Offset": abs(apply_steer),
    "HCA_01_LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_01_Vib_Freq": 18,
    "HCA_01_Sendestatus": 1 if lkas_enabled else 0,
    "EA_ACC_Wunschgeschwindigkeit": 327.36,
  }
  return packer.make_can_msg("HCA_01", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control):
  values = ldw_stock_values.copy()

  values.update({
    "LDW_Status_LED_gelb": 1 if enabled and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if enabled and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible,
    "LDW_Lernmodus_rechts": 3 if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = gra_stock_values.copy()

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "LS_Abbrechen": cancel,
    "LS_Tip_Wiederaufnahme": resume,
  })

  return packer.make_can_msg("LS_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  # TODO:
  return 0


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  commands = []

  # TODO
  """values = {
    "ACS_Sta_ADR": acc_control,
    "ACS_StSt_Info": acc_enabled,
    "ACS_Typ_ACC": acc_type,
    "ACS_Anhaltewunsch": acc_type == 1 and stopping,
    "ACS_FreigSollB": acc_enabled,
    "ACS_Sollbeschl": accel if acc_enabled else 3.01,
    "ACS_zul_Regelabw": 0.2 if acc_enabled else 1.27,
    "ACS_max_AendGrad": 3.0 if acc_enabled else 5.08,
  }

  commands.append(packer.make_can_msg("ACC_System", bus, values))"""

  return commands


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  # TODO
  return 0


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance):
  """values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": distance + 2,
    "ACC_Display_Prio": 3,
    "ACC_Abstandsindex": lead_distance,
  }

  return packer.make_can_msg("ACC_02", bus, values)"""
  return 0


def volkswagen_mlb_checksum(address: int, sig, d: bytearray) -> int:
  xor_starting_value = {
    0x109: 0x08, # ACC_01
    0x30C: 0x0F, # ACC_02
    0x324: 0x27, # ACC_04
    0x10D: 0x0C, # ACC_05
  }
  if address in xor_starting_value:
    return xor_checksum(address, sig, d, xor_starting_value[address])
  else:
    return crc8h2f_checksum(address, sig, d)
