const addr_checks carnival_rx_checks = {
  .check = NULL,
  .len = 0,
};

int carnival_rx_hook(CANPacket_t *to_push) {
  UNUSED(to_push);
  return true;
}

// *** all carnival output safety mode ***

// Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa
const uint16_t CARNIVAL_PARAM_PASSTHROUGH = 1;
bool carnival_passthrough = false;

static const addr_checks* carnival_init(int16_t param) {
  UNUSED(param);
  carnival_passthrough = GET_FLAG(param, CARNIVAL_PARAM_PASSTHROUGH);
  controls_allowed = true;
  relay_malfunction_reset();
  return &carnival_rx_checks;
}

static int carnival_tx_hook(CANPacket_t *to_send) {
  UNUSED(to_send);
  return true;
}

static int carnival_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return true;
}

static int carnival_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  UNUSED(to_fwd);
  int bus_fwd = -1;

  int addr = GET_ADDR(to_fwd);
  if (carnival_passthrough) {
    if (bus_num == 0) {
      bus_fwd = 2;
    }
    if ((bus_num == 2) && (addr != 0x12a) &&  (addr != 0x1e0)) {   // KIA KA4  298, 480
    //if ((bus_num == 2)) {   // KIA KA4
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

const safety_hooks carnival_hooks = {
  .init = carnival_init,
  .rx = carnival_rx_hook,
  .tx = carnival_tx_hook,
  .tx_lin = carnival_tx_lin_hook,
  .fwd = carnival_fwd_hook,
};
