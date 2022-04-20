import copy
from cereal import car
from common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.landrover.values import DBC, STEER_THRESHOLD


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEAR_PRND"]["GEAR_SHIFT"]

  def update(self, cp, cp_cam):

    ret = car.CarState.new_message()

    self.frame = int(cp.vl["THROTTLE"]["COUNTER"])

    ret.doorOpen = (cp.vl["SEAT_BELT"]["SEAT_BELT_DRIVER"]  == 0)
    ret.seatbeltUnlatched = (cp.vl["SEAT_BELT"]["SEAT_BELT_DRIVER"]  == 0)

    ret.brakePressed = (cp.vl["CRUISE_CONTROL"]['DRIVER_BRAKE'] == 1)
    ret.brake = 0
    ret.gas = 0
    ret.gasPressed = ret.gas > 1e-3

    #ret.wheelSpeeds = (cp.vl["SPEED_01"]["SPEED01"] + cp.vl["SPEED_02"]["SPEED02"]) / 2.
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["SPEED_04"]["WHEEL_SPEED_FL"],
      cp.vl["SPEED_04"]["WHEEL_SPEED_FR"],
      cp.vl["SPEED_03"]["WHEEL_SPEED_RL"],
      cp.vl["SPEED_03"]["WHEEL_SPEED_RR"],
      unit=1,
    )

    ret.vEgoRaw = (cp.vl["SPEED_01"]["SPEED01"] + cp.vl["SPEED_02"]["SPEED02"]) / 2.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["TURN_SIGNAL"]['LEFT_BLINK'],cp.vl["TURN_SIGNAL"]['RIGHT_BLINK'])
    ret.leftTurn, ret.rightTurn = self.update_blinker_from_lamp(50, cp.vl["TURN_SIGNAL"]['LEFT_TURN'],cp.vl["TURN_SIGNAL"]['RIGHT_TURN'])

    ret.steeringAngleDeg = cp.vl["EPS_01"]["STEER_ANGLE01"]
    ret.steeringRateDeg = cp.vl["EPS_01"]["STEER_SPEED01"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["GEAR_PRND"]["GEAR_SHIFT"], None))

    ret.cruiseState.enabled = cp.vl["CRUISE_CONTROL"]["CRUISE_ON"] == 1  # ACC is green.
    ret.cruiseState.available = ret.cruiseState.enabled   # FIXME: for now same as enabled
    ret.cruiseState.speed = cp.vl["CRUISE_CONTROL"]['SPEED_CRUISE_RESUME'] * CV.KPH_TO_MS
    # ret.cruiseState.standstill = False

    ret.steeringTorque = cp.vl["EPS_02"]["STEER_TORQUE_DRIVER02"]
    ret.steeringTorqueEps = cp.vl["EPS_02"]["STEER_TORQUE_MOTOR02"] / 10.  # scale to Nm
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    steer_state = 1 #cp.vl[""]["LKAS_STATE"]
    #ret.steerFaultPermanent = steer_state == 4 or (steer_state == 0 and ret.vEgo > self.CP.minSteerSpeed)
    ret.steerFaultPermanent = False #steer_state == 4 or (steer_state == 0 and ret.vEgo > self.CP.minSteerSpeed)

    self.lkas_counter = cp_cam.vl["LKAS_RUN"]['COUNTER']
    self.lkas_run = copy.copy(cp_cam.vl["LKAS_RUN"])
    self.lkas_status = copy.copy(cp_cam.vl["LKAS_STATUS"])

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("COUNTER", "THROTTLE"),
      ("STEER_RATE00", "EPS_00"),
      ("STEER_ANGLE01", "EPS_01"),
      ("STEER_SPEED01", "EPS_01"),
      ("STEER_TORQUE_DRIVER02", "EPS_02"),
      ("STEER_TORQUE_MOTOR02", "EPS_02"),
      ("STEER_TORQUE_DRIVER03", "EPS_03"),
      ("STEER_TQ", "EPS_04"),
      ("GEAR_SHIFT", "GEAR_PRND"),
      ("CRUISE_ON", "CRUISE_CONTROL"),
      ("DRIVER_BRAKE", "CRUISE_CONTROL"),
      ("SPEED_CRUISE_RESUME", "CRUISE_CONTROL"),
      ("ACCELATOR_DRIVER", "ACCELATOR_DRIVER"),
      ("SEAT_BELT_DRIVER", "SEAT_BELT"),
      ("RIGHT_TURN", "TURN_SIGNAL"),
      ("LEFT_TURN", "TURN_SIGNAL"),
      ("LEFT_BLINK", "TURN_SIGNAL"),
      ("RIGHT_BLINK", "TURN_SIGNAL"),
      ("SPEED01", "SPEED_01"),
      ("SPEED02", "SPEED_02"),
      #("LEFT_ALERT_1", "LEFT_ALERT", 0),
      #("RIGHT_ALERT_1", "RIGHT_ALERT", 0),
      #( "COUNTER", "LKAS_RUN"),
      ("WHEEL_SPEED_FR", "SPEED_04"),
      ("WHEEL_SPEED_FL", "SPEED_04"),
      ("WHEEL_SPEED_RR", "SPEED_03"),
      ("WHEEL_SPEED_RL", "SPEED_03"),
      ("GREEN2WHITE_RIGHT", "LKAS_STATUS"),
      ("GREEN2WHITE_LEFT", "LKAS_STATUS"),
      ("FRONT_CAR_DISTANCE", "LKAS_HUD_STAT"),
      ("LADAR_DISTANCE", "LKAS_HUD_STAT"),
      ("HIBEAM", "HEAD_LIGHT"),
    ]

    # It's considered invalid if it is not received for 10x the expected period (1/f).
    checks = [
      # sig_address, frequency
      ("EPS_00", 100),
      ("EPS_01", 100),
      ("EPS_02", 100),
      ("EPS_03", 100),
      ("EPS_04", 100),
      ("SPEED_01", 100),
      ("SPEED_02", 100),
      ("SPEED_03", 100),
      ("SPEED_04", 100),
      ("CRUISE_CONTROL", 100),
      ("SEAT_BELT", 100),
      ("LKAS_HUD_STAT", 00),
      ("HEAD_LIGHT", 100),
      ("LKAS_STATUS", 100),
      ("GEAR_PRND", 100),
      ("TURN_SIGNAL", 10),
      ("SEAT_BELT", 10),
      ("ACCELATOR_DRIVER", 10),
      ( "THROTTLE", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
      signals = [
        # sig_name, sig_address, default
        # TODO read in all the other values
        ( "ALLFFFF", "LKAS_RUN"),
        ( "A1", "LKAS_RUN"),
        ( "HIGH_TORQ", "LKAS_RUN"),
        ( "ALL11", "LKAS_RUN"),
        ( "COUNTER", "LKAS_RUN"),
        ( "STEER_TORQ", "LKAS_RUN"),
        ( "LKAS_GREEN", "LKAS_RUN"),
      ]
      checks = [
         ( "LKAS_RUN", 1),
      ]

      return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)
