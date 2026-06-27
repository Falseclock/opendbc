"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.aBasis = 0.0

    # Palisade 2023 non-HDA2 TSR over-speed corridor: raw 8-byte mirrors of the
    # camera's CAM_TSR (0x4EC) and LKAS12 (0x53E) frames, plus the integer
    # speed limit currently shown on the cluster (native unit — km/h or mph
    # depending on coding). The carcontroller rebroadcasts both frames with our
    # alarm-corridor bits overridden; everything else is preserved verbatim.
    self.tsr_lkas12_raw = bytearray(8)
    self.tsr_cam_tsr_raw = bytearray(8)
    self.tsr_displayed_limit = 0

  def update_speed_limit(self, cp, cp_cam) -> float:
    speed_limit = 0

    if self.CP.flags & HyundaiFlags.CANFD:
      if self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE:
        bus = cp if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG else cp_cam
        speed_limit = bus.vl["FR_CMR_02_100ms"]["ISLW_SpdCluMainDis"]
    else:
      nav, cam = 0, 0
      if self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE:
        nav = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"]
      if self.CP_SP.flags & HyundaiFlagsSP.HAS_LKAS12:
        cam = cp_cam.vl["LKAS12"]["CF_Lkas_TsrSpeed_Display_Clu"]

      speed_limit = cam if cam not in (0, 255) else nav

    if speed_limit in (0, 255):
      speed_limit = 0

    return speed_limit

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser], speed_conv: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS13"]["aBasis"]

    if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC:
      cruise_msg = "LABEL11" if self.CP.flags & HyundaiFlags.EV else \
                   "E_CRUISE_CONTROL" if self.CP.flags & HyundaiFlags.HYBRID else \
                   "EMS16"
      cruise_available_sig = "CC_React" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_M"
      cruise_enabled_sig = "CC_ACT" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_S"
      cruise_speed_msg = "E_EMS11" if self.CP.flags & HyundaiFlags.EV else \
                         "ELECT_GEAR" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "LVR12"
      cruise_speed_sig = "Cruise_Limit_Target" if self.CP.flags & HyundaiFlags.EV else \
                         "SLC_SET_SPEED" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "CF_Lvr_CruiseSet"
      ret.cruiseState.available = cp.vl[cruise_msg][cruise_available_sig] != 0
      ret.cruiseState.enabled = cp.vl[cruise_msg][cruise_enabled_sig] != 0
      ret.cruiseState.speed = cp.vl[cruise_speed_msg][cruise_speed_sig] * speed_conv
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False

      if not self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_NO_FCA:
        cp_cruise = cp if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_RADAR_FCA else cp_cam

        aeb_src = "FCA11"
        aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
        aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src]["FCA_CmdAct"] != 0
        ret.stockFcw = aeb_warning and not aeb_braking
        ret.stockAeb = aeb_warning and aeb_braking

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_conv

    if self.CP.carFingerprint == CAR.HYUNDAI_PALISADE_2023 and self.CP.flags & HyundaiFlags.CAN_CANFD_BLENDED:
      self._update_tsr_raw(cp_cam)

  def _update_tsr_raw(self, cp_cam: CANParser) -> None:
    # Reconstruct the 8 bytes of LKAS12 (0x53E) and CAM_TSR (0x4EC) from their
    # decoded signals so the carcontroller can re-emit them with our override
    # bits applied. All 64 bits of each frame are covered by the DBC, so the
    # mirror is lossless.
    lk = cp_cam.vl["LKAS12"]
    self.tsr_displayed_limit = int(lk["CF_Lkas_TsrSpeed_Display_Clu"])
    self.tsr_lkas12_raw[0] = int(lk["CHECKSUM"]) & 0xFF
    self.tsr_lkas12_raw[1] = (
      (int(lk["CF_Lkas_CountryCode"]) << 0) |
      (int(lk["COUNTER"])             << 4)
    ) & 0xFF
    self.tsr_lkas12_raw[2] = int(lk["CF_Lkas_Byte2"]) & 0xFF
    self.tsr_lkas12_raw[3] = int(lk["CF_Lkas_TsrSpeed_Display_Clu"]) & 0xFF
    self.tsr_lkas12_raw[4] = int(lk["CF_Lkas_TsrSpeed_Display_Navi"]) & 0xFF
    self.tsr_lkas12_raw[5] = (
      (int(lk["CF_Lkas_DawStatus"])        << 0) |
      (int(lk["CF_Lkas_SpeedLimitOffset"]) << 3) |
      (int(lk["CF_Lkas_SpeedLimitWarn"])   << 6) |
      (int(lk["CF_Lkas_SignProjection"])   << 7)
    ) & 0xFF
    self.tsr_lkas12_raw[6] = (
      (int(lk["CF_Lkas_SpeedSignAttention"]) << 0) |
      (int(lk["CF_Lkas_IslaMessage"])        << 3) |
      (int(lk["CF_Lkas_Byte6_Bit54_55"])     << 6)
    ) & 0xFF
    self.tsr_lkas12_raw[7] = (
      (int(lk["CF_Lkas_SignDetected"])    << 0) |
      (int(lk["CF_Lkas_SlaState"])        << 5) |
      (int(lk["CF_Lkas_Byte7_Bit62_63"])  << 6)
    ) & 0xFF

    cam = cp_cam.vl["CAM_TSR"]
    self.tsr_cam_tsr_raw[0] = int(cam["TSR_Byte0"]) & 0xFF
    self.tsr_cam_tsr_raw[1] = (
      (int(cam["TSR_SpeedLimitCondition"]) << 0) |
      (int(cam["TSR_Byte1_HighNibble"])    << 4)
    ) & 0xFF
    self.tsr_cam_tsr_raw[2] = int(cam["TSR_Byte2"]) & 0xFF
    self.tsr_cam_tsr_raw[3] = int(cam["TSR_Speed_Limit"]) & 0xFF
    self.tsr_cam_tsr_raw[4] = (
      (int(cam["TSR_State_LowNibble"])    << 0) |
      (int(cam["TSR_OverSpeedLimitWarn"]) << 4) |
      (int(cam["TSR_SpeedLimitChanged"])  << 5) |
      (int(cam["TSR_State_HighBits"])     << 6)
    ) & 0xFF
    self.tsr_cam_tsr_raw[5] = int(cam["TSR_Byte5"]) & 0xFF
    self.tsr_cam_tsr_raw[6] = int(cam["TSR_Byte6"]) & 0xFF
    self.tsr_cam_tsr_raw[7] = int(cam["TSR_Byte7"]) & 0xFF

  def update_canfd_ext(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser],
                       speed_factor: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS"]["aBasis"]

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_factor
