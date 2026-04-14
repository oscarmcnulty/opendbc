import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class HCAMitigation:
  """
  Manages HCA fault mitigations for VW/Audi EPS racks:
    * Reduces torque by 1 for a single frame after commanding the same torque value for too long
    * For MLB racks: proactively disables HCA during low-torque phases before the 6-minute EPS
      timer expires, resetting the timer without a jarring forced cutout
  """

  def __init__(self, CCP, eps_timer_workaround=False):
    self._max_same_torque_frames = CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP)
    self._same_torque_frames = 0

    self._eps_timer_workaround = eps_timer_workaround
    if eps_timer_workaround:
      self._steer_step = CCP.STEER_STEP
      self._timer_running_frames = 0
      self._timer_resetting_frames = 0
      self._low_torque_frames = 0
      self._frames_for_bm = CCP.STEER_TIME_BM / DT_CTRL
      self._frames_for_low_torque = CCP.STEER_TIME_LOW_TORQUE / DT_CTRL
      self._frames_for_reset = CCP.STEER_TIME_RESET / DT_CTRL
      self._frames_for_alert = CCP.STEER_TIME_ALERT / DT_CTRL
      self._low_torque_threshold = CCP.STEER_LOW_TORQUE

  @property
  def eps_timer_soft_disable_alert(self):
    if not self._eps_timer_workaround:
      return False
    return self._timer_running_frames > self._frames_for_alert

  def update(self, apply_torque, apply_torque_last, hca_enabled):
    # Stuck-torque mitigation: nudge torque by 1 after commanding same value too long
    if apply_torque != 0 and apply_torque_last == apply_torque:
      self._same_torque_frames += 1
      if self._same_torque_frames > self._max_same_torque_frames:
        apply_torque -= (1, -1)[apply_torque < 0]
        self._same_torque_frames = 0
    else:
      self._same_torque_frames = 0

    # MLB 6-minute timer mitigation: proactively disable HCA during low-torque phases
    # once we're past STEER_TIME_BM (~4 min), so the EPS timer resets without a forced cutout
    if self._eps_timer_workaround:
      if hca_enabled:
        self._timer_running_frames += self._steer_step
        if self._timer_running_frames >= self._frames_for_bm:
          if abs(apply_torque) <= self._low_torque_threshold:
            self._low_torque_frames += self._steer_step
            if self._low_torque_frames >= self._frames_for_low_torque:
              hca_enabled = False
              apply_torque = 0
          else:
            self._low_torque_frames = 0
            if self._timer_resetting_frames > 0:
              # Still within a reset window that didn't fully complete; keep torque zero
              apply_torque = 0
      else:
        self._low_torque_frames = 0

      if not hca_enabled:
        self._timer_resetting_frames += self._steer_step
        if self._timer_resetting_frames >= self._frames_for_reset:
          self._timer_running_frames = 0
          self._timer_resetting_frames = 0
      else:
        self._timer_resetting_frames = 0

    return apply_torque, hca_enabled


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    else:
      self.CCS = mqbcan

    self.apply_torque_last = 0
    self.torque_output_can_last = 0
    self.gra_acc_counter_last = None
    self.eps_timer_soft_disable_alert = False
    self.hca_mitigation = HCAMitigation(self.CCP, eps_timer_workaround=bool(CP.flags & VolkswagenFlags.MLB))

    self.last_set_speed = 0
    self.last_lead_distance_bars = 0
    self.mlb_hud_text = 0
    self.texte_timer = 0
    self.accel_last = 0.0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      apply_torque = 0
      if CC.latActive:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)

      hca_enabled = apply_torque != 0
      apply_torque, hca_enabled = self.hca_mitigation.update(apply_torque, self.apply_torque_last, hca_enabled)
      self.eps_timer_soft_disable_alert = self.hca_mitigation.eps_timer_soft_disable_alert

      torque_output_can = apply_torque if hca_enabled else 0
      self.apply_torque_last = apply_torque
      self.torque_output_can_last = torque_output_can
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, torque_output_can, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl:
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
        if self.CP.flags & VolkswagenFlags.MLB:
          # Jerk-limit the accel setpoint to match ACC_neg/pos_Sollbeschl_Grad (4.0 m/s³) declared in ACC_01.
          # It may be possible to increase ACC_neg/pos_Sollbeschl_Grad instead of doing this. Either was the ECU is faulting during high jerk.
          dt = DT_CTRL * self.CCP.ACC_CONTROL_STEP
          accel = float(np.clip(accel, self.accel_last - 4.0 * dt, self.accel_last + 4.0 * dt))
        self.accel_last = accel
        stopping = actuators.longControlState == LongCtrlState.stopping
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.longActive, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation))

      #if self.aeb_available:
      #  if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
      #  if self.frame % self.CCP.AEB_HUD_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      # FIXME: PQ may need to use the on-the-wire mph/kmh toggle to fix rounding errors
      # FIXME: Detect clusters with vEgoCluster offsets and apply an identical vCruiseCluster offset
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH

      # MLB:Logic for hud text, bottom acc text display
      if self.CP.flags & VolkswagenFlags.MLB:
        if set_speed != self.last_set_speed:
          self.texte_timer = self.frame + int(2.0 / DT_CTRL)
          self.mlb_hud_text = 21
          self.last_set_speed = set_speed
        elif hud_control.leadDistanceBars != self.last_lead_distance_bars:
          self.texte_timer = self.frame + int(2.0 / DT_CTRL)
          self.mlb_hud_text = {1: 2, 2: 3, 3: 4, 4: 5}.get(hud_control.leadDistanceBars, 0)
          self.last_lead_distance_bars = hud_control.leadDistanceBars
        elif self.frame > self.texte_timer:
          self.mlb_hud_text = 0

      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                       lead_distance, hud_control, self.mlb_hud_text))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.torque_output_can_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
