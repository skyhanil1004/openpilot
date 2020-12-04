#!/usr/bin/env python3
from cereal import car
from selfdrive.car.landrover.values import CAR
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.swaglog import cloudlog

class CarInterface(CarInterfaceBase):
  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=None, car_fw=None):
    if fingerprint is None:
      fingerprint = gen_empty_fingerprint()

    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "landrover"
    ret.safetyModel = car.CarParams.SafetyModel.landrover

    ret.communityFeature = True
    #ret.enableCruise = True

    ret.wheelbase = 2.922  # in meters for rangerover vogue  2017
    ret.steerRatio = 11.2 # Pacifica Hybrid 2017 is 16.2
    ret.mass = 2500. + STD_CARGO_KG  # kg curb weight Pacifica Hybrid 2017
    #tire_stiffness_factor = 0.5371   # hand-tun
    ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.01,0.02], [0.0005,0.001]]
    ret.lateralTuning.pid.kf = 0.00001   # full torque for 10 deg at 80mph means 0.00007818594
    ret.steerActuatorDelay = 0.025       # 0.05 -> 0.065 12/11 modify org 0.1 -> 0.05 then good straight
    ret.steerRateCost = 0.5              # org 0.7
    ret.steerLimitTimer = 0.4

    ret.centerToFront = ret.wheelbase * 0.44
    ret.minSteerSpeed = 1.8    # m/s
    #ret.minEnableSpeed = -1.   # enable is done by stock ACC, so ignore this
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)
    ret.enableCamera = True

    #ret.steerRatioRear = 0.
    #ret.steerLimitTimer = 0.4
    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)

    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid

    # speeds
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # events
    events = self.create_common_events(ret, extra_gears=[car.CarState.GearShifter.low],
                                       gas_resume_speed=2.)

    if ret.vEgo < self.CP.minSteerSpeed:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    # copy back carState packet to CS
    self.CS.out = ret.as_reader()

    return self.CS.out

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):
 
    if (self.CS.frame == -1):
      return []  # if we haven't seen a frame 220, then do not update.

    #can_sends = self.CC.update(c.enabled, self.CS, c.actuators, c.cruiseControl.cancel, c.hudControl.visualAlert)
    can_sends = self.CC.update(c.enabled, self.CS, self.frame,
                               c.actuators, c.cruiseControl.cancel, c.hudControl.visualAlert)
    self.frame += 1

    return can_sends
