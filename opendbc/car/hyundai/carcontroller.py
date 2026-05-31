import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR
from opendbc.car.interfaces import CarControllerBase

from opendbc.sunnypilot.car.hyundai.escc import EsccCarController
from opendbc.sunnypilot.car.hyundai.icbm import IntelligentCruiseButtonManagementInterface
from opendbc.sunnypilot.car.hyundai.longitudinal.controller import LongitudinalController
from opendbc.sunnypilot.car.hyundai.lead_data_ext import LeadDataCarController
from opendbc.sunnypilot.car.hyundai.mads import MadsCarController

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController(CarControllerBase, EsccCarController, LeadDataCarController, LongitudinalController, MadsCarController,
                    IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    EsccCarController.__init__(self, CP, CP_SP)
    MadsCarController.__init__(self)
    LeadDataCarController.__init__(self, CP)
    LongitudinalController.__init__(self, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0

    self.accel_last = 0
    self.apply_torque_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0

    # TSR approaching-limit audible alert prototype (Palisade 2023 non-HDA2 only).
    # Fires our own 2-second beep when cluster speed reaches (TSR_Speed_Limit - 2);
    # native camera over-speed alert remains independent and fires later on real over-speed.
    self.is_palisade_2023_non_hda2 = (CP.carFingerprint == CAR.HYUNDAI_PALISADE_2023
                                      and bool(CP.flags & HyundaiFlags.CAN_CANFD_BLENDED))
    self.tsr_approach_margin = 2     # km/h before TSR_Speed_Limit
    self.tsr_beep_duration = 200     # frames; 100 Hz cycle → 2 seconds
    self.tsr_in_approach_zone = False
    self.tsr_beep_until_frame = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    EsccCarController.update(self, CS)
    LeadDataCarController.update(self, CC_SP)
    MadsCarController.update(self, self.CP, CC, CC_SP, self.frame)
    if self.frame % 5 == 0:
      LongitudinalController.update(self, CC, CS)

    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)

    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, MAX_ANGLE_FRAMES,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    if not CC.latActive:
      apply_torque = 0

    # Hold torque with induced temporary fault when cutting the actuation bit
    # FIXME: we don't use this with CAN FD?
    torque_fault = CC.latActive and not apply_steer_req

    self.apply_torque_last = apply_torque

    # accel + longitudinal
    accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    can_sends = []

    # *** common hyundai stuff ***

    # Common shared configuration
    can_canfd_blended = bool(self.CP.flags & HyundaiFlags.CAN_CANFD_BLENDED)

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not ((self.CP.flags & (HyundaiFlags.CANFD_CAMERA_SCC)) or self.ESCC.enabled) and \
            self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if (self.CP.flags & (HyundaiFlags.CANFD | HyundaiFlags.CAN_CANFD_BLENDED)) else 0
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    # *** CAN/CAN FD specific ***
    if self.CP.flags & HyundaiFlags.CANFD:
      can_sends.extend(self.create_canfd_msgs(apply_steer_req, apply_torque, set_speed_in_units, accel,
                                              stopping, hud_control, CS, CC))
    else:
      can_sends.extend(self.create_can_msgs(apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel,
                                            stopping, hud_control, actuators, CS, CC, can_canfd_blended))

    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CS, CC_SP, self.packer, self.frame, self.last_button_frame, self.CAN))

    # Earlier attempts (kept commented for revert):
    #
    # ATTEMPT 1: artificial chime via 0x4EC byte 4 bit 4 OR'd inside an approach zone.
    # Disabled because cluster did not actually play the sound on real route, even
    # though the bit pulsed.
    # if self.is_palisade_2023_non_hda2 and self.frame % 10 == 0:
    #   limit = CS.displayed_speed_limit
    #   if limit > 0:
    #     in_zone = CS.cluster_speed >= (limit - self.tsr_approach_margin)
    #     if in_zone and not self.tsr_in_approach_zone:
    #       self.tsr_beep_until_frame = self.frame + self.tsr_beep_duration
    #     self.tsr_in_approach_zone = in_zone
    #   else:
    #     self.tsr_in_approach_zone = False
    #   our_warn = self.frame < self.tsr_beep_until_frame
    #   data = bytearray(CS.cam_tsr_raw)
    #   if our_warn:
    #     data[4] |= 0x10
    #   can_sends.append((0x4EC, bytes(data), self.CAN.ECAN))
    #
    # ATTEMPT 2: LKAS12 SpdLimOffset injection (forced Enabled=1, Value=4 = +5 km/h).
    # On the real car this *did* make the HU show the offset menu with +5 selected
    # by default — and also flipped the camera into KR-style sign recognition. But
    # selecting offset in the menu errored ("vehicle not responding" — HU was trying
    # to write coding back to camera via UDS and we don't reply), TSR_Speed_Limit
    # displayed at the cluster did NOT shift by the offset (camera publishes the
    # unchanged byte 3 because it doesn't know we faked the LKAS12 enable), and the
    # OverSpeedLimitWarn bit pulsed without audible chime. So the cluster does not
    # honor LKAS12 SpdLimOffset by itself — only the MFC does, when it sets it itself.
    # if self.is_palisade_2023_non_hda2 and self.frame % 10 == 0 and any(CS.lkas12_raw):
    #   data = bytearray(CS.lkas12_raw)
    #   data[1] |= 0x01                          # CF_Lkas_SpdLimOffsetEnabled = 1
    #   data[5] = (data[5] & ~0x38) | (4 << 3)   # CF_Lkas_SpdLimOffsetValue   = 4 (+5 km/h)
    #   data[0] = hyundaican.hyundai_checksum(bytes(data[1:8]))
    #   can_sends.append((0x53E, bytes(data), self.CAN.ECAN))

    # ATTEMPT 3 (active): rewrite TSR_Speed_Limit in 0x4EC byte 3 directly.
    # Panda's hyundai_fwd_hook blocks the camera's 0x4EC bus2→bus0 so we re-emit
    # a full byte-for-byte mirror at 10 Hz with byte 3 (TSR_Speed_Limit) shifted
    # by +5 km/h. The cluster will display the shifted value and use it natively
    # for its own over-speed comparison — so the cluster's native over-speed chime
    # fires when actual_speed > (camera_limit + 5). No fake bits, no LKAS12 lie,
    # no UDS write expected from HU. byte 3 = 0 is preserved as 0 (no limit recognized).
    if self.is_palisade_2023_non_hda2 and self.frame % 10 == 0 and any(CS.cam_tsr_raw):
      data = bytearray(CS.cam_tsr_raw)
      if data[3] > 0:
        data[3] = min(data[3] + 5, 0xFF)
      can_sends.append((0x4EC, bytes(data), self.CAN.ECAN))

    # ATTEMPT 4 (active alongside ATTEMPT 3): inject ONLY CF_Lkas_SpdLimOffsetValue
    # = 4 (= +5 km/h step) into LKAS12 byte 5 bits 3..5. CF_Lkas_SpdLimOffsetEnabled
    # (byte 1 bit 0) is NOT touched — left at the camera's value (0 by default on
    # RU coding). Test goal: does the cluster apply the offset when only Value is
    # set (Enabled = 0)? If yes, cluster will display (base + 5) and trigger its
    # native over-speed chime — without the side effects of ATTEMPT 2 (no HU offset
    # menu, no UDS error on selection, no camera KR-flip). Other LKAS12 bytes are
    # mirrored verbatim from camera; CHECKSUM (byte 0) is recomputed because our
    # edit invalidates camera's original (CRC-8 J1850 over bytes 1..7).
    if self.is_palisade_2023_non_hda2 and self.frame % 10 == 0 and any(CS.lkas12_raw):
      data = bytearray(CS.lkas12_raw)
      data[5] = (data[5] & ~0x38) | (4 << 3)   # CF_Lkas_SpdLimOffsetValue = 4
      data[0] = hyundaican.hyundai_checksum(bytes(data[1:8]))
      can_sends.append((0x53E, bytes(data), self.CAN.ECAN))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.accel = self.tuning.actual_accel

    self.frame += 1
    return new_actuators, can_sends

  def create_can_msgs(self, apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel, stopping, hud_control, actuators, CS, CC, can_canfd_blended):
    can_sends = []

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    if can_canfd_blended:
      can_sends.extend(hyundaican.create_lkas11_can_canfd_blended(self.packer, self.frame, self.CP, apply_torque, apply_steer_req,
                                                                  torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                                  hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                                  left_lane_warning, right_lane_warning,
                                                                  self.lkas_icon, CS.msg_364))
    else:
      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_torque, apply_steer_req,
                                                torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                left_lane_warning, right_lane_warning,
                                                self.lkas_icon))

    # Button messages
    if not self.CP.openpilotLongitudinalControl:
      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame

    if self.CP.openpilotLongitudinalControl and can_canfd_blended and not self.ESCC.enabled:
      can_sends.extend(hyundaican.create_radar_aux_messages(self.packer, self.CAN, self.frame))

    if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
      # TODO: unclear if this is needed
      jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
      use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
      if can_canfd_blended:
        can_sends.extend(hyundaican.create_acc_commands_can_canfd_blended(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                      self.lead_data, hud_control, set_speed_in_units, stopping,
                                                      CC.cruiseControl.override, use_fca, self.CP,
                                                      CS.main_cruise_enabled, self.tuning, self.CAN, CS.out.vEgo, self.ESCC))
      else:
        can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                      self.lead_data, hud_control, set_speed_in_units, stopping,
                                                      CC.cruiseControl.override, use_fca, self.CP,
                                                      CS.main_cruise_enabled, self.tuning, self.ESCC))


    # 20 Hz LFA MFA message
    if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
      can_sends.append(hyundaican.create_lfahda_mfc(self.packer, self.frame, self.CP, CC.enabled, self.lfa_icon))

    # 5 Hz ACC options
    if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl and not can_canfd_blended:
      can_sends.extend(hyundaican.create_acc_opt(self.packer, self.CP, self.CAN, self.ESCC))

    # 2 Hz front radar options
    if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl and not self.ESCC.enabled and not can_canfd_blended:
      can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    return can_sends

  def create_canfd_msgs(self, apply_steer_req, apply_torque, set_speed_in_units, accel, stopping, hud_control, CS, CC):
    can_sends = []

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl

    # steering control
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_torque, self.lkas_icon))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    if self.frame % 5 == 0 and lka_steering:
      can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg,
                                                        self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT))

    # LFA and HDA icons
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long):
      can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled, self.lfa_icon))

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, CC.leftBlinker, CC.rightBlinker))

    if self.CP.openpilotLongitudinalControl:
      if lka_steering:
        can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
      else:
        can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
      if self.frame % 2 == 0:
        can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                         set_speed_in_units, hud_control, self.lead_data, CS.main_cruise_enabled, self.tuning))
        self.accel_last = accel
    else:
      # button presses
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

    return can_sends