from cereal import car
from selfdrive.car.landrover.values import DBC, STEER_THRESHOLD, CAR
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.conversions import Conversions as CV
from common.params import Params

GearShifter = car.CarState.GearShifter


def get_can_parser_landrover(CP):
  signals = [
    # sig_name, sig_address, default
    ("STEER_RATE00", "EPS_00", 0),
    ("STEER_ANGLE01", "EPS_01", 0),
    ("STEER_SPEED01", "EPS_01", 0),
    ("STEER_TORQUE_DRIVER02", "EPS_02", 0),
    ("STEER_TORQUE_MOTOR02", "EPS_02", 0),
    ("STEER_TORQUE_DRIVER03", "EPS_03", 0),
    ("STEER_TQ", "EPS_04", 0),
    ("GEAR_SHIFT", "GEAR_PRND", 0),
    ("CRUISE_ON", "CRUISE_CONTROL", 0),
    ("DRIVER_BRAKE", "CRUISE_CONTROL", 0),
    ("SPEED_CRUISE_RESUME", "CRUISE_CONTROL", 1),
    ("ACCELATOR_DRIVER", "ACCELATOR_DRIVER", 0),
    ("SEAT_BELT_DRIVER", "SEAT_BELT", 0),
    ("RIGHT_TURN", "TURN_SIGNAL", 0),
    ("LEFT_TURN", "TURN_SIGNAL", 0),
    ("RIGHT_BLINK", "TURN_SIGNAL", 0),
    ("LEFT_BLINK", "TURN_SIGNAL", 0),
    ("BLINK_SIGNAL", "TURN_SIGNAL", 0),
    ("COUNTER", "LKAS_RUN", -1),
    ("SPEED01", "SPEED_01", 0),
    ("SPEED02", "SPEED_02", 0),
    ("LEFT_ALERT_1", "LEFT_ALERT", 0),
    ("RIGHT_ALERT_1", "RIGHT_ALERT", 0),
    ("WHEEL_SPEED_FR", "SPEED_04", 0),
    ("WHEEL_SPEED_FL", "SPEED_04", 0),
    ("WHEEL_SPEED_RR", "SPEED_03", 0),
    ("WHEEL_SPEED_RL", "SPEED_03", 0),
    ("GREEN2WHITE_RIGHT", "LKAS_STATUS", 1),
    ("GREEN2WHITE_LEFT", "LKAS_STATUS", 1),
    ("FRONT_CAR_DISTANCE", "LKAS_HUD_STAT", 0),
    ("LADAR_DISTANCE", "LKAS_HUD_STAT", 0),
    ("HIBEAM", "HEAD_LIGHT", 0),
  ]

# It's considered invalid if it is not received for 10x the expected period (1/f).
  checks = [
    # sig_address, frequency
    ("EPS_00", 0),
    ("EPS_01", 0),
    ("EPS_02", 0),
    ("EPS_03", 0),
    ("EPS_04",  0),
    ("SPEED_01", 0),
    ("SPEED_02", 0),
    ("SPEED_03", 0),
    ("SPEED_04", 0),
    ("CRUISE_CONTROL", 1),
    ("SEAT_BELT", 0),
    ("LKAS_HUD_STAT", 0),
    ("HEAD_LIGHT", 0),
    ("LKAS_STATUS", 0),
    ("TURN_SIGNAL", 0),
  ]

  return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

def get_cam_can_parser_landrover(CP):
    signals = [
      # sig_name, sig_address, default
      # TODO read in all the other values
      ( "ALLFFFF", "LKAS_RUN", 0),
      ( "A1", "LKAS_RUN", 0),
      ( "HIGH_TORQ", "LKAS_RUN", 0),
      ( "ALL11", "LKAS_RUN", 0),
      ( "COUNTER", "LKAS_RUN", -1),
      ( "STEER_TORQ", "LKAS_RUN", 0),
      ( "LKAS_GREEN", "LKAS_RUN", 0),
    ]

    checks = []

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)

def parse_gear_shifter(can_gear):
  if can_gear == 0x0:
    return "P"
  elif can_gear == 0x9:
    return "R"
  elif can_gear == 0x12:
    return "N"
  elif can_gear == 0x1b:
    return "D"
  elif can_gear == 0xbb:
    return "S"
  return "unknown"


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEAR_PRND"]["GEAR_SHIFT"]

    #Auto detection for setup
    self.no_radar = True
    self.has_lfa_hda = CP.hasLfaHda
    self.leftBlinker = False
    self.rightBlinker = False
    self.cruise_main_button = 0
    self.mdps_error_cnt = 0
    self.cruise_unavail_cnt = 0

    self.apply_steer = 0.

    # scc smoother
    self.acc_mode = False
    self.cruise_gap = 1
    self.brake_pressed = False
    self.gas_pressed = False
    self.standstill = False
    self.cruiseState_enabled = False
    self.cruiseState_speed = 0

    self.use_cluster_speed = False
    self.long_control_enabled = False
    self.mdps_bus = 2

  def update(self, cp, cp2, cp_cam):
    cp_mdps = cp

    self.prev_cruise_buttons = self.cruise_buttons
    self.prev_cruise_main_button = self.cruise_main_button
    self.prev_left_blinker = self.leftBlinker
    self.prev_right_blinker = self.rightBlinker

    ret = car.CarState.new_message()

    ret.seatbeltUnlatched = (cp.vl["SEAT_BELT"]["SEAT_BELT_DRIVER"]  == 0)

    self.is_set_speed_in_mph = False
    self.speed_conv_to_ms = CV.MPH_TO_MS if self.is_set_speed_in_mph else CV.KPH_TO_MS

    cluSpeed = cp.vl["SPEED_01"]["SPEED01"]

    ret.cluSpeedMs = cluSpeed * self.speed_conv_to_ms

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["SPEED_01"]["SPEED01"],
      cp.vl["SPEED_02"]["SPEED02"],
      cp.vl["SPEED_01"]["SPEED01"],
      cp.vl["SPEED_02"]["SPEED02"],
    )

    vEgoRawWheel = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    vEgoWheel, aEgoWheel = self.update_speed_kf(vEgoRawWheel)

    vEgoRawClu = cluSpeed * self.speed_conv_to_ms
    vEgoClu, aEgoClu = self.update_clu_speed_kf(vEgoRawClu)

    if self.use_cluster_speed:
      ret.vEgoRaw = vEgoRawClu
      ret.vEgo = vEgoClu
      ret.aEgo = aEgoClu
    else:
      ret.vEgoRaw = vEgoRawWheel
      ret.vEgo = vEgoWheel
      ret.aEgo = aEgoWheel

    ret.vCluRatio = (vEgoWheel / vEgoClu) if (vEgoClu > 3. and vEgoWheel > 3.) else 1.0

    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringRateDeg = cp.vl["EPS_01"]["STEER_SPEED01"]
    ret.steeringAngleDeg = cp.vl["EPS_01"]["STEER_ANGLE01"]

    # TODO find
    #ret.aBasis = cp.vl["TCS13"]["aBasis"]  ??
    #ret.yawRate = cp.vl["ESP12"]["YAW_RATE"]
    ret.steeringTorqueEps = cp_mdps.vl["EPS_02"]["STEER_TORQUE_DRIVER02"] / 10.  # scale to Nm

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["TURN_SIGNAL"]["LEFT_BLINK"],cp.vl["TURN_SIGNAL"]["RIGHT_BLINK"])
    #ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(50, cp.vl["TURN_SIGNAL"]["LEFT_TURN"],cp.vl["TURN_SIGNAL"]["RIGHT_TURN"])

    ret.steeringTorque = cp.vl["EPS_02"]["STEER_TORQUE_DRIVER02"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    if self.CP.enableAutoHold:
      ret.autoHold = cp.vl["ESP11"]["AVH_STAT"]

    # cruise state
    ret.cruiseState.enabled = cp.vl["CRUISE_CONTROL"]["CRUISE_ON"] == 1
    ret.cruiseState.available = True
    ret.cruiseState.standstill = False

    ret.cruiseState.enabledAcc = ret.cruiseState.enabled

    if ret.cruiseState.enabled:
      ret.cruiseState.speed = cp_scc.vl["SCC11"]["VSetDis"] * self.speed_conv_to_ms if not self.no_radar else \
                                         cp.vl["LVR12"]["CF_Lvr_CruiseSet"] * self.speed_conv_to_ms
    else:
      ret.cruiseState.speed = 0
    #self.cruise_main_button = cp.vl["CLU11"]["CF_Clu_CruiseSwMain"]
    #self.cruise_buttons = cp.vl["CLU11"]["CF_Clu_CruiseSwState"]

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = (cp.vl["CRUISE_CONTROL"]["DRIVER_BRAKE"] == 1)
    ret.brakeHoldActive = 0
    ret.parkingBrake = 0

    # TODO: Check this
    ret.brakeLights = ret.brakePressed
    ret.gasPressed = ret.gas > 1e-3
    ret.gas = 0

    # TODO: refactor gear parsing in function
    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    gear = cp.vl["GEAR_PRND"]["GEAR_SHIFT"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    ret.leftBlindspot = False
    ret.rightBlindspot = False

    self.lkas_run = copy.copy(cp_cam.vl["LKAS_RUN"])
    self.lkas_status = copy.copy(cp_cam.vl["LKAS_STATUS"])

    self.lkas_counter = cp_cam.vl["LKAS_RUN"]['COUNTER']
    return ret


  @staticmethod
  def get_can_parser(CP):
     return get_can_parser_landrover(CP)

  @staticmethod
  def get_cam_can_parser(CP):
     return get_cam_can_parser_landrover(CP)
