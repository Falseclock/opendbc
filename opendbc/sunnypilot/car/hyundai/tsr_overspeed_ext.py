"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import math

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.hyundaican import hyundai_checksum
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import CAR, HyundaiFlags

# Live config comes from openpilot params (registered in params_keys.h, synced by sunnylink, editable
# from the settings UIs). opendbc also runs standalone (tests, tools) where openpilot isn't importable —
# there the defaults below apply, which reproduce the original hardcoded behavior (fixed +5 km/h, real
# speed, chime on).
try:
  from openpilot.common.params import Params
except ImportError:
  Params = None

# type: 0 off (stock camera behavior) | 1 fixed | 2 percentage; source: 0 real vEgo | 1 cluster speed
CONFIG_DEFAULTS = {"type": 1, "source": 0, "kph": 5, "mph": 3, "pct": 5, "chime": 1}
CONFIG_KEYS = {"type": "HkgTsrAlarmOffsetType", "source": "HkgTsrAlarmSource", "kph": "HkgTsrAlarmOffsetKph",
               "mph": "HkgTsrAlarmOffsetMph", "pct": "HkgTsrAlarmOffsetPct", "chime": "HkgTsrAlarmChime"}
CONFIG_REFRESH_FRAMES = 100  # re-read params at 1 Hz (100 Hz carcontroller) so changes apply mid-drive

# Phase timing in seconds since the over-speed condition latched. Cluster receives
# the corridor bits at 10 Hz so phase boundaries land on a tick.
TX_PERIOD_FRAMES = 10  # 10 Hz at the carcontroller's 100 Hz update rate
T_BLINK_START = 3.0
T_CHIME_START = 6.0
T_BLINK_END = 9.0


class TsrOverSpeedCarController:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.enabled = (CP.carFingerprint == CAR.HYUNDAI_PALISADE_2023
                    and bool(CP.flags & HyundaiFlags.CAN_CANFD_BLENDED))
    self.CAN = CanBus(CP) if self.enabled else None

    # Frame counter where the current over-speed event began; -1 = IDLE.
    self.over_start_frame = -1
    # Last seen displayed limit; used to detect tighter-limit re-trigger.
    self.prev_limit = 0
    # Own COUNTER for LKAS12 (replaces camera's value so the cluster sees an
    # unbroken sequence regardless of source-frame jitter).
    self.lkas12_cnt = 0

    self.params = Params() if (self.enabled and Params is not None) else None
    self.config = dict(CONFIG_DEFAULTS)
    self._refresh_config()

  def _refresh_config(self):
    if self.params is None:
      return
    for name, key in CONFIG_KEYS.items():
      try:
        self.config[name] = int(self.params.get(key, return_default=True))
      except (TypeError, ValueError):
        self.config[name] = CONFIG_DEFAULTS[name]

  def update(self, frame, CS):
    # Returns list of (addr, bytes, bus) tuples to extend can_sends with.
    if not self.enabled or frame % TX_PERIOD_FRAMES != 0:
      return []
    if frame % CONFIG_REFRESH_FRAMES == 0:
      self._refresh_config()
    if self.config["type"] == 0:
      # Off: openpilot doesn't own the corridor — the camera's stock frames pass through untouched.
      self.over_start_frame = -1
      return []
    if not any(CS.tsr_lkas12_raw) or not any(CS.tsr_cam_tsr_raw):
      return []  # wait for first camera frame to land in carstate

    # Baseline is configurable. Real speed (default) compares against true wheel-derived vEgo — Hyundai
    # speedometers over-read ~5% by regulation, so the margin acts on real km/h regardless of speed.
    # Cluster speed compares against the biased dial value the driver actually sees. Ceil for stricter
    # rounding on the real baseline.
    if self.config["source"] == 1:
      speed = int(CS.cluster_speed)
    else:
      speed_conv = CV.MS_TO_MPH if not CS.is_metric else CV.MS_TO_KPH
      speed = math.ceil(CS.out.vEgo * speed_conv)
    limit = int(CS.tsr_displayed_limit)

    # Threshold: fixed margin in the CAR's cluster unit (CF_Clu_SPEED_UNIT decides which param applies),
    # or a percentage of the displayed limit.
    if self.config["type"] == 2:
      threshold = limit * (1.0 + self.config["pct"] / 100.0)
    else:
      threshold = limit + (self.config["kph"] if CS.is_metric else self.config["mph"])
    over = limit > 0 and speed > threshold

    if not over:
      self.over_start_frame = -1
    elif self.over_start_frame < 0:
      self.over_start_frame = frame
    elif 0 < limit < self.prev_limit:
      # Limit tightened while still over → restart the cycle from t=0.
      self.over_start_frame = frame
    self.prev_limit = limit

    red, blink, chime = 0, 0, 0
    if self.over_start_frame >= 0:
      elapsed = (frame - self.over_start_frame) / 100.0
      if elapsed < T_BLINK_START:
        red = 1
      elif elapsed < T_CHIME_START:
        red, blink = 1, 1
      elif elapsed < T_BLINK_END:
        red, blink, chime = 1, 1, 1
      else:
        red = 1
    if not self.config["chime"]:
      chime = 0   # visual-only mode: red + blink phases run, the audible phase is muted

    return [
      self._build_lkas12(CS, red, blink),
      self._build_cam_tsr(CS, chime),
    ]

  def _build_lkas12(self, CS, red, blink):
    data = bytearray(CS.tsr_lkas12_raw)
    # byte 1: keep CountryCode (low nibble), replace COUNTER (high nibble) with ours
    data[1] = (data[1] & 0x0F) | (self.lkas12_cnt << 4)
    # byte 5 bit 6: CF_Lkas_SpeedLimitWarn (red sign coloring)
    data[5] = (data[5] & ~0x40) | ((red & 0x1) << 6)
    # byte 6 bits 0-2: CF_Lkas_SpeedSignAttention (1 = blink_warn, 0 = none)
    data[6] = (data[6] & ~0x07) | (0x1 if blink else 0x0)
    # byte 0: CHECKSUM (CRC-8 J1850 over bytes 1..7 with our overrides applied)
    data[0] = hyundai_checksum(bytes(data[1:8]))
    self.lkas12_cnt = (self.lkas12_cnt + 1) & 0xF
    return (0x53E, bytes(data), self.CAN.ECAN)

  def _build_cam_tsr(self, CS, chime):
    data = bytearray(CS.tsr_cam_tsr_raw)
    # byte 4 bit 4: TSR_OverSpeedLimitWarn (cluster chime trigger)
    data[4] = (data[4] & ~0x10) | ((chime & 0x1) << 4)
    return (0x4EC, bytes(data), self.CAN.ECAN)
