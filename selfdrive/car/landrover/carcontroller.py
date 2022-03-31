from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.landrover.values import CarControllerParams, CAR
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.p = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)

    self.apply_steer_last = 0
    self.CP = CP
    self.steer_rate_limited = False
    self.last_resume_frame = 0
    self.accel = 0

    self.controls_allowed = True
    self.apply_steer_last = 0
    self.lkascnt = 0
    self.prev_frame = -1
    self.hud_count = 0
    self.alert_active = False
    self.send_chime = False
    self.gone_fast_yet = False


  def update(self, c, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, hud_speed,
             left_lane, right_lane, left_lane_depart, right_lane_depart):

    # Steering Torque
    new_steer = int(round(actuators.steer * self.p.STEER_MAX))
    #new_steer = int(round(actuators.steer * 150))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
    self.steer_rate_limited = new_steer != apply_steer

    # disable when temp fault is active, or below LKA minimum speed
    lkas_active = enabled and not CS.out.steerFaultTemporary and CS.out.vEgo >= CS.CP.minSteerSpeed

    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    can_sends = []

    for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
       if(frame % fr_step == 0):  # 25 0.25s period
           can_sends.append(make_can_msg(addr, vl, bus ))

    if (frame % 20 == 0):
       can_sends.append(create_lkas_hud(self.packer, left_lane, right_lane, left_lane_depart, right_lane_depart ))

    if (frame % 2 == 0):
       new_msg = create_lkas_command(self.packer, int(apply_steer), self.gone_fast_yet, self.lkascnt)
       self.lkascnt += 1
       #cloudlog.warning("LANDROVER LKAS (%s)", new_msg)
       can_sends.append(new_msg)

    self.prev_frame = frame

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.p.STEER_MAX

    return new_actuators, can_sends
