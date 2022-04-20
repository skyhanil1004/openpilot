#!/usr/bin/env python3
from cereal import car
from panda import Panda
from common.params import Params
from common.conversions import Conversions as CV
from selfdrive.car.landrover.values import CAR, CarControllerParams
#from selfdrive.car.landrover.radar_interface import RADAR_START_ADDR
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase

class CarInterface(CarInterfaceBase):
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, disable_radar=False):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "landrover"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.landrover, 0)]
    ret.radarOffCan = True #RADAR_START_ADDR not in fingerprint[1]

    ret.openpilotLongitudinalControl = False #Params().get_bool("DisableRadar") and (candidate not in LEGACY_SAFETY_MODE_CAR)
    ret.pcmCruise = False #not ret.openpilotLongitudinalControl

    ret.dashcamOnly = False #candidate in {CAR.KIA_OPTIMA_H, CAR.ELANTRA_GT_I30}

    ret.steerRatio = 15.5
    ret.steerActuatorDelay = 0.011      # 0.05 -> 0.065 12/11 modify org 0.1 -> 0.025 then good straight
    ret.lateralTuning.pid.kf = 0.00005   # full torque for 10 deg at 80mph means 0.00007818594

    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 20.], [9., 20.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.0001,0.0002], [0.000005,0.00001]]

    ret.steerRateCost = 0.5
    ret.steerLimitTimer = 0.4
    tire_stiffness_factor = 0.5371   # hand-tun

    ret.stoppingControl = True
    ret.vEgoStopping = 1.0

    ret.longitudinalTuning.kpV = [0.1]
    ret.longitudinalTuning.kiV = [0.0]
    ret.stopAccel = 0.0
    ret.longitudinalActuatorDelayUpperBound = 1.0 # s

    ret.mass = 2500. + STD_CARGO_KG  # kg curb weight Pacifica Hybrid 2017
    ret.wheelbase =  2.922  # in meters for rangerover vogue  2017
    ret.minSteerSpeed = 1.8    # m/s
    ret.minEnableSpeed = -1.   # enable is done by stock ACC, so ignore this

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)
                                                                         # tire_stiffness_factor=tire_stiffness_factor)
    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # events
    events = self.create_common_events(ret, extra_gears=[car.CarState.GearShifter.low])

    if ret.vEgo < self.CP.minSteerSpeed:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):

    if (self.CS.frame == -1):
      return car.CarControl.Actuators.new_message(), []  # if we haven't seen a frame 220, then do not update.

    return self.CC.update(c.enabled, self.CS, c.actuators, c.cruiseControl.cancel, c.hudControl.visualAlert)

