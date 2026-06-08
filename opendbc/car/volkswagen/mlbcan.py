from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.mqbcan import (crc8h2f_checksum,xor_checksum,
                                           create_lka_hud_control as mqb_create_lka_hud_control)

# The drivetrain coordinator only honors ACC_01 deceleration requests above ~15 km/h. Below that we
# command the ESP directly through the ANB (Automatische Notbremsung) channel in ACC_10 to brake to a
# stop. Gate exactly at the ACC_01 floor so the two never request braking at the same time (below the
# floor the ECU ignores ACC_01, above it ACC_10 stays inactive). Tune against vehicle data.
ACC_10_MAX_SPEED = 15 * CV.KPH_TO_MS
ACC_10_STANDSTILL_SPEED = 0.5  # m/s, request the AWV standstill hold once essentially stopped

# TODO: Parameterize the hca control type (5 vs 7) and consolidate with MQB (and PQ?)
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
  return mqb_create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = {s: gra_stock_values[s] for s in [
    "LS_Hauptschalter",           # ACC button, on/off
    "LS_Typ_Hauptschalter",       # ACC main button type
    "LS_Codierung",               # ACC button configuration/coding
    "LS_Tip_Stufe_2",             # unknown related to stalk type
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "LS_Abbrechen": cancel,
    "LS_Tip_Wiederaufnahme": resume,
  })

  return packer.make_can_msg("LS_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  if acc_faulted:
    acc_control = 6
  elif long_active:
    acc_control = 3
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold, v_ego):
  commands = []

  acc_05_values = {
    "ACC_Freigabe_Momentenanf": 1 if accel > 0 else 0, # increased acceleration requested?
    "ACC_Freigabe_Verzanf": 1 if accel < 0 else 0, # decreased acceleration requested?
    "ACC_Getriebestellung_P": 0,
    "ACC_limitierte_Anfahrdyn": 0,
    "ACC_Momentenanforderung": int(accel*100) if accel > 0 else 0, # "torque requested",
    "ACC_zul_Regelabw": 0,
    "ACC_Verz_anf": accel if accel < 0 else 0, # brake accel requested
    "ACC_Loeseanforderung": starting, # 1 when starting again from stop
    "ACC_StartStopp_Info": starting, # 1 when moving, 0 when stopped
    "ACC_Vorbefuellung_Bremsanlage": 1 if accel < 0 else 0,
    "ACC_ax_Getriebe": accel, # positive or negative accel requested
    "ACC_Status_ACC": acc_control,
    "ACC_Betaetigung_EPB": 0,
    "ACC_Beeinflussung_ESP": 0,
    "ACC_Anhalten": stopping,
    "ACC_KD_Fehler": 0,
  }
  commands.append(packer.make_can_msg("ACC_05", bus, acc_05_values))

  # Below the ACC_01 floor the drivetrain coordinator faults on a longitudinal request, so keep
  # ACC_01's request fully inactive there and hand all longitudinal authority to ACC_10 (ANB). ACC
  # engagement status is left untouched so the system stays engaged through the handoff.
  acc_01_active = acc_enabled and v_ego >= ACC_10_MAX_SPEED
  acc_01_values = {
    "ACC_Status_ACC": acc_control,
    "ACC_Sollbeschleunigung": accel if acc_01_active else 0,
    "ACC_zul_Regelabw_unten": 0.2,
    "ACC_zul_Regelabw_oben": 0.2,
    "ACC_neg_Sollbeschl_Grad": 4.0 if acc_01_active else 0,
    "ACC_pos_Sollbeschl_Grad": 4.0 if acc_01_active else 0,
    "ACC_Anfahren": starting if acc_01_active else 0,
    "ACC_Anhalten": stopping if acc_01_active else 0,
    "ACC_Dynamik": 2,
    "ACC_Minimale_Bremsung": stopping if acc_01_active else 0,
  }
  commands.append(packer.make_can_msg("ACC_01", bus, acc_01_values))

  # ACC_10 (ANB): low-speed deceleration request straight to the ESP, used below the ACC_01 floor so
  # openpilot can continue braking to a full stop. Once stopped, AWV_Halten asks the ESP to clamp and
  # hold the car at standstill (a timed pre sense-style hold); we keep ANB braking until the ESP
  # confirms the hold, then release it and rely on AWV_Halten. Quiescent heartbeat otherwise.
  holding = acc_enabled and esp_hold
  request_hold = acc_enabled and (esp_hold or (stopping and v_ego < ACC_10_STANDSTILL_SPEED))
  low_speed_braking = acc_enabled and accel < 0 and v_ego < ACC_10_MAX_SPEED and not holding
  acc_10_values = {
    # A bare ANB_Zielbremsung request is ignored by the ESP (it releases the brakes within ~0.5s).
    # Mimic the stock AEB sequence the ESP expects: assert the collision-mitigation context and brake
    # prefill, and request both partial and target braking carrying the deceleration. The decel value
    # is shared by both releases (ANB_Zielbrems_Teilbrems_Verz_Anf).
    # TODO: prefill/CM context may need to lead the brake by a few frames; tune against vehicle data.
    "ANB_CM_Info": 1 if low_speed_braking else 0,               # CM configuration/available
    "ANB_CM_Anforderung": 1 if low_speed_braking else 0,        # CM request to ESP
    "AWV1_Anf_Prefill": 1 if low_speed_braking else 0,          # brake prefill (pre-pressurize)
    "ANB_Zielbrems_Teilbrems_Verz_Anf": accel if low_speed_braking else 0,
    "ANB_Teilbremsung_Freigabe": 1 if low_speed_braking else 0,  # partial braking release
    "ANB_Zielbremsung_Freigabe": 1 if low_speed_braking else 0,  # target braking release (controlled stop)
    "AWV_Halten": 1 if request_hold else 0,                      # request ESP hold at standstill
    "AWV1_ECD_Anlauf": 1 if request_hold else 0,                 # spin up ESC pump to maintain clamp
  }
  commands.append(packer.make_can_msg("ACC_10", bus, acc_10_values))

  return commands


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  # TODO: happens to resemble the ACC control value for now, but extend this for init/gas override later
  return acc_control_value(main_switch_on, acc_faulted, long_active)


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, hud_control, mlb_hud_text):

  acc_active = True if acc_hud_status in (3,4) else False
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.04,
    "ACC_Display_Prio": 0,
    "ACC_Anzeige_Zeitluecke": 1 if acc_active else 0,
    "ACC_Gesetzte_Zeitluecke": hud_control.leadDistanceBars, # TODO: Update openpilot charisma using stock rocker switch
    "ACC_Tachokranz": 1 if acc_active else 0,
    "ACC_Relevantes_Objekt": 2 if hud_control.visualAlert > 0 else (1 if acc_active and hud_control.leadVisible else 0),
    "ACC_Status_Prim_Anz": 2 if hud_control.visualAlert > 0 else (1 if acc_active else 0),
    "ACC_Akustik": 1 if hud_control.audibleAlert == 5 else 0, # Audible alert on OP warningImmediate
    "ACC_Abstandsindex": 1023 if acc_active else 1022,
    "ACC_Texte_Primaeranz": mlb_hud_text,
  }

  return packer.make_can_msg("ACC_02", bus, values)


def volkswagen_mlb_checksum(address: int, sig, d: bytearray) -> int:

  if address in {0x9F, 0x117, 0x126}: # LH_EPS_03, ACC_10, HCA_01
    return crc8h2f_checksum(address, sig, d)

  seed = (address & 0xFF) - 1

  if address == 0x397: # LDW_02
    seed = seed - 2
  elif address in {0x102, 0x106, 0x10E}: #Getriebe_03, ESP_05, TSK_04
    seed = seed + 2
  elif address in {0x30C, 0x324}: # ACC_02, ACC_04
    seed = seed + 4

  return xor_checksum(address, sig, d, initial_value=(seed & 0xFF))
