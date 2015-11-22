// For ECE2031 project
// Written by Michael Reilly

#include "SComp.hpp"

namespace TFC {

constexpr const std::array<double, 8> SComp::sonar_angles;
constexpr const std::array<Line2D, 6> SComp::arena;

SComp::SComp(std::vector<int> const& memory)
  : memory(memory), state(State::RESET_PC), pc_stack(),
    m_io_dist({{no_echo, no_echo, no_echo, no_echo, no_echo, no_echo, no_echo,
    no_echo}}) {
  std::array<int, std::mt19937::state_size> seed_data;
  std::random_device r;
  std::generate_n(seed_data.data(), seed_data.size(), std::ref(r));
  std::seed_seq seq(std::begin(seed_data), std::end(seed_data));

  engine = std::mt19937_64(seq);
}

void SComp::stepInstruction(size_t count) {
  for (size_t i = 0; i < count; ++i) {
    if (state != State::FETCH && state != State::RESET_PC) {
      throw std::exception();
    }

    do {
      runState();
    } while (state != State::FETCH);
  }
}

std::vector<int> SComp::getMemory() const {
  return memory;
}
  
void SComp::setSwitches(uint16_t bitmask) {
  m_io_switches = bitmask;
}

uint16_t SComp::getRedLeds() const {
  return m_io_redLeds;
}

uint16_t SComp::getTimer() const {
  return m_io_timer;
}

uint16_t SComp::getXIO() const {
  return m_io_xio;
}

uint16_t SComp::getSevenSeg1() const {
  return m_io_sevenSeg1;
}

uint16_t SComp::getSevenSeg2() const {
  return m_io_sevenSeg2;
}

uint16_t SComp::getLCD() const {
  return m_io_lcd;
}

uint16_t SComp::getXLeds() const {
  return m_io_xleds;
}

uint16_t SComp::getBeep() const {
  return m_io_beep;
}

uint16_t SComp::getCTimer() const {
  return m_io_ctimer;
}

uint16_t SComp::getUARTData() const {
  return m_io_uart_dat;
}

uint16_t SComp::getUARTRdy() const {
  return m_io_uart_rdy;
}

std::array<uint16_t, 8> SComp::getSonarDist() const {
  return m_io_dist;
}

uint16_t SComp::getSonalarmDist() const {
  return m_io_sonalarm_distance;
}

uint16_t SComp::getSonalarmFlags() const {
  return m_io_sonalarm_flags;
}

uint16_t SComp::getSonarInt() const {
  return m_io_sonarint;
}

uint16_t SComp::getSonarEn() const {
  return m_io_sonaren;
}

uint16_t SComp::getOdometryLPos() const {
  return m_io_lpos;
}

uint16_t SComp::getOdometryLVel() const {
  return m_io_lvel;
}

uint16_t SComp::getLVelCmd() const {
  return m_io_lvelcmd;
}

uint16_t SComp::getOdometryRPos() const {
  return m_io_rpos;
}

uint16_t SComp::getOdometryRVel() const {
  return m_io_rvel;
}

uint16_t SComp::getRVelCmd() const {
  return m_io_rvelcmd;
}

uint16_t SComp::getOdometryXPos() const {
  return m_io_xpos;
}

uint16_t SComp::getOdometryYPos() const {
  return m_io_ypos;
}

uint16_t SComp::getOdometryTheta() const {
  return m_io_theta;
}

Vec2D SComp::getTruePos() const {
  return true_pos;
}

double SComp::getTrueHeading() const {
  return true_heading;
}

double SComp::getTrueLVel() const {
  return true_lvel;
}

double SComp::getTrueRVel() const {
  return true_rvel;
}

void SComp::runState() {
  ++executed_instructions;

  if (mw) {
    memory[mem_addr] = ac;
  } else {
    mdr = memory[mem_addr];
  }

  io_addr = ir & 0xFF;

  updateIO();

  mem_addr = (state == State::FETCH) ? pc : ir & address_mask;
  io_cycle = (state == State::EX_IN ||
              state == State::EX_OUT2);

  io_write = io_write_int;

  // This is one half of the check, the other half is in sendInterrupt
  for (int i = 0; i < (int)Interrupt::SIZE; ++i) {
    auto bitmask = 1 << i;
    if ((int_ack & bitmask) || !(iie & bitmask)) {
      int_req &= ~bitmask; // unsets bit
    }
  }

  int_req_sync = int_req;

  switch (state) {
    case State::RESET_PC:
      reset();
      break;
		case State::FETCH:
      fetchAndHandleInt();
      break;
    case State::DECODE:
      decode();
      break;
		case State::EX_LOAD:
      load();
      break;
		case State::EX_STORE:
      store();
      break;
		case State::EX_STORE2:
      store2();
      break;
		case State::EX_ADD:
      add();
      break;
		case State::EX_SUB:
      sub();
      break;
		case State::EX_JUMP:
      jump();
      break;
		case State::EX_JNEG:
      jneg();
      break;
		case State::EX_JPOS:
      jpos();
      break;
		case State::EX_JZERO:
      jzero();
      break;
		case State::EX_AND:
      and_();
      break;
		case State::EX_OR:
      or_();
      break;
		case State::EX_XOR:
      xor_();
      break;
		case State::EX_SHIFT:
      shift();
      break;
		case State::EX_ADDI:
      addi();
      break;
		case State::EX_ILOAD:
      iload();
      break;
		case State::EX_ISTORE:
      istore();
      break;
		case State::EX_CALL:
      call();
      break;
		case State::EX_RETURN:
      return_();
      break;
		case State::EX_IN:
      in();
      break;
		case State::EX_OUT:
      out();
      break;
		case State::EX_OUT2:
      out2();
      break;
		case State::EX_LOADI:
      loadi();
      break;
		case State::EX_RETI:
      reti();
      break;
    default:
      throw std::exception();
      break;
  }
}

// We're going to perform IO updates prior to setting io_data, just because
void SComp::updateIO() {
  doIoWrite();
  doIoRead();
  doIoUpdate();
}

void SComp::doIoWrite() {
  m_io_resetpos = 0;
  if (io_write && io_cycle) {
    switch ((IOAddr)io_addr) {
      case IOAddr::Switches:
        // Do nothing
        break;
      case IOAddr::Leds:
        m_io_redLeds = io_data;
        break;
      case IOAddr::Timer:
        m_io_timer = 0;
        break;
      case IOAddr::Xio:
        // Do Nothing
        break;
      case IOAddr::Sseg1:
        m_io_sevenSeg1 = io_data;
        break;
      case IOAddr::Sseg2:
        m_io_sevenSeg2 = io_data;
        break;
      case IOAddr::Lcd:
        m_io_lcd = io_data;
        break;
      case IOAddr::Xleds:
        m_io_xleds = io_data;
        break;
      case IOAddr::Beep:
        m_io_beep = io_data;
        break;
      case IOAddr::Ctimer:
        m_io_ctimer = io_data;
        break;
      case IOAddr::Lpos:
        // Do nothing
        break;
      case IOAddr::Lvel:
        // Do nothing
        break;
      case IOAddr::Lvelcmd:
        m_io_lvelcmd = io_data;
        break;
      case IOAddr::Rpos:
        // Do nothing
        break;
      case IOAddr::Rvel:
        // Do nothing
        break;
      case IOAddr::Rvelcmd:
        m_io_rvelcmd = io_data;
        break;
      case IOAddr::I2c_cmd:
        m_io_i2c_cmd = io_data;
        break;
      case IOAddr::I2c_data:
        m_io_i2c_data = io_data;
        break;
      case IOAddr::I2c_rdy:
        m_io_i2c_rdy = io_data;
        break;
      case IOAddr::Uart_dat:
        m_io_uart_dat = io_data;
        break;
      case IOAddr::Uart_rdy:
        // Do nothing
        break;
      case IOAddr::Sonar:
        // TODO: Does writing to this do anything?
        //m_io_sonar = io_data;
        break;
      case IOAddr::Dist0:
        // Do nothing
        break;
      case IOAddr::Dist1:
        // Do nothing
        break;
      case IOAddr::Dist2:
        // Do nothing
        break;
      case IOAddr::Dist3:
        // Do nothing
        break;
      case IOAddr::Dist4:
        // Do nothing
        break;
      case IOAddr::Dist5:
        // Do nothing
        break;
      case IOAddr::Dist6:
        // Do nothing
        break;
      case IOAddr::Dist7:
        // Do nothing
        break;
      case IOAddr::Sonalarm:
        m_io_sonalarm_distance = io_data;
        break;
      case IOAddr::Sonarint:
        m_io_sonarint = io_data;
        break;
      case IOAddr::Sonaren:
        m_io_sonaren = io_data;
        break;
      case IOAddr::Xpos:
        // Do Nothing
        break;
      case IOAddr::Ypos:
        // Do Nothing
        break;
      case IOAddr::Theta:
        // Do Nothing
        break;
      case IOAddr::Resetpos:
        m_io_resetpos = 1;
        break;
      case IOAddr::Rin:
        // Do Nothing
        // Not implemented
        break;
      case IOAddr::Lin:
        // Do Nothing
        // Not implemented
        break;
      default:
        // do nothing
        break;
    }
  }
}

void SComp::doIoRead() {
  if (!io_write && io_cycle) {
    switch ((IOAddr)io_addr) {
      case IOAddr::Switches:
        io_data = m_io_switches;
        break;
      case IOAddr::Leds:
        // Do nothing
        break;
      case IOAddr::Timer:
        io_data = m_io_timer;
        break;
      case IOAddr::Xio:
        io_data = m_io_xio;
        break;
      case IOAddr::Sseg1:
        // Do nothing
        break;
      case IOAddr::Sseg2:
        // Do nothing
        break;
      case IOAddr::Lcd:
        // Do nothing
        break;
      case IOAddr::Xleds:
        // Do nothing
        break;
      case IOAddr::Beep:
        // Do nothing
        break;
      case IOAddr::Ctimer:
        // Do nothing
        break;
      case IOAddr::Lpos:
        io_data = m_io_lpos;
        break;
      case IOAddr::Lvel:
        io_data = m_io_lvel;
        break;
      case IOAddr::Lvelcmd:
        // Do nothing
        break;
      case IOAddr::Rpos:
        io_data = m_io_rpos;
        break;
      case IOAddr::Rvel:
        io_data = m_io_rvel;
        break;
      case IOAddr::Rvelcmd:
        // Do nothing
        break;
      case IOAddr::I2c_cmd:
        // Do nothing
        break;
      case IOAddr::I2c_data:
        // Do nothing
        break;
      case IOAddr::I2c_rdy:
        // Do nothing
        break;
      case IOAddr::Uart_dat:
        // Do nothing Not implemented
        break;
      case IOAddr::Uart_rdy:
        // Do nothing
        break;
      case IOAddr::Sonar:
        // TODO: Does reading from this do anything?
        // io_data = m_io_sonar;
        break;
      case IOAddr::Dist0:
        io_data = m_io_dist[0];
        break;
      case IOAddr::Dist1:
        io_data = m_io_dist[1];
        break;
      case IOAddr::Dist2:
        io_data = m_io_dist[2];
        break;
      case IOAddr::Dist3:
        io_data = m_io_dist[3];
        break;
      case IOAddr::Dist4:
        io_data = m_io_dist[4];
        break;
      case IOAddr::Dist5:
        io_data = m_io_dist[5];
        break;
      case IOAddr::Dist6:
        io_data = m_io_dist[6];
        break;
      case IOAddr::Dist7:
        io_data = m_io_dist[7];
        break;
      case IOAddr::Sonalarm:
        io_data = m_io_sonalarm_flags;
        break;
      case IOAddr::Sonarint:
        // Do nothing
        break;
      case IOAddr::Sonaren:
        // Do nothing
        break;
      case IOAddr::Xpos:
        io_data = m_io_xpos;
        break;
      case IOAddr::Ypos:
        io_data = m_io_ypos;
        break;
      case IOAddr::Theta:
        io_data = m_io_theta;
        break;
      case IOAddr::Resetpos:
        // Do nothing
        break;
      case IOAddr::Rin:
        // Do Nothing
        // Not implemented
        break;
      case IOAddr::Lin:
        // Do Nothing
        // Not implemented
        break;
      default:
        // do nothing
        break;
    }
  }
}

void SComp::doIoUpdate() {
  updateTimers();
  updatePos();
  updateSonar();
  updateI2C();
  updateUART();
}

void SComp::updateTimers() {
  size_t cclockPeriod = instruction_period / 100; // 100 Hz
  size_t clockPeriod = instruction_period / 10; // 10 Hz
  if (executed_instructions % cclockPeriod == 0) {
    if (executed_instructions % clockPeriod == 0) {
      ++m_io_timer;
    }
    ++m_io_ctimer;
  }
}
  
void SComp::updatePos() {
  // Does not simulate gear lash because I cannot do it in an intelligent way
  // and an algorithm could potentially find a "weak spot" in the simulation and
  // exploit it in a way that would be unrealistic

  // Additionally, this is all very fudged.  I don't try very hard to emulate
  // the behavior of the conversion, I just sin/cos myself and round

  // Period for odometry update
  size_t clockPeriod = instruction_period / 100; // 100 Hz

  if (executed_instructions % clockPeriod) {
    return;
  }

  if (m_io_resetpos) {
    m_io_lpos = 0;
    m_io_rpos = 0;
    m_io_xpos = 0;
    m_io_ypos = 0;
    m_io_theta = 0;

    pos = {};
    heading = 0;

    // don't touch true_pos and true_heading
  }

  std::normal_distribution<double> randSpeed(0, speed_stdev);
  auto getDisp = [&randSpeed, this](double vel) {
      double mm_per_centisec = speed_slope * vel + speed_inter;
      return max(mm_per_centisec + randSpeed(engine), 0.);
    };

  double leftWheelDisp = getDisp(true_lvel);
  m_io_lpos += leftWheelDisp;

  double rightWheelDisp = getDisp(true_rvel);
  m_io_rpos += rightWheelDisp;

  // update both
  std::tie(pos, heading) = circleMotion(pos, heading, leftWheelDisp, rightWheelDisp);
  std::tie(true_pos, true_heading) = circleMotion(true_pos, true_heading, leftWheelDisp, rightWheelDisp);

  // fuzz the true_pos and true_heading
  std::uniform_real_distribution<double> randPos(-pos_fuzz, pos_fuzz);
  std::uniform_real_distribution<double> randHead(-heading_fuzz, heading_fuzz);
  true_pos += randPos(engine);
  true_heading += randHead(engine);

  // Send the data out to the stuff
  m_io_xpos = round(pos.x() / ticks_per_mm);
  m_io_ypos = round(pos.y() / ticks_per_mm);
  m_io_theta = round(heading * 180/M_PI);

  // Finally update the velocities
  std::normal_distribution<double> randAccel(0, accel_fuzz);

  if (true_rvel != m_io_rvelcmd) {
    true_rvel = approach<double>(m_io_rvelcmd, true_rvel, max_accel + randAccel(engine));
  }

  if (true_lvel != m_io_lvelcmd) {
    true_lvel = approach<double>(m_io_lvelcmd, true_lvel, max_accel + randAccel(engine));
  }

  // Period for m_io_lvel and m_io_rvel update
  size_t clockPeriod10 = instruction_period / 10; // 10 Hz

  // This isn't accurate, it's kind of... too accurate
  if (executed_instructions % clockPeriod10 == 0) {
    m_io_rvel = round(true_rvel);
    m_io_lvel = round(true_lvel);
  }
}

void SComp::updateSonar() {
  size_t clockPeriod = instruction_period / 25; // 25Hz

  if (executed_instructions % clockPeriod) {
    return;
  }

  // Clear disabled sonars first
  for (int i = 0; i < 8; ++i) {
    // If this sonar is disabled
    if (!((1 << i) & m_io_sonaren)) {
      // set it to no_echo
      m_io_dist[i] = no_echo;
    }
  }

  for (int i = 1; i < 9; ++i) { // starting from the next sonar
    int this_sonar = (last_sonar_used + i) % 8;
    uint16_t bitmask = 1 << (this_sonar);
    if (bitmask & m_io_sonaren) {
      last_sonar_used = this_sonar;
      auto position = true_pos;
      auto angle = true_heading + sonar_angles[this_sonar];

      // because the sonars are located on the side of the robot
      position += Vec2D::withAngle(angle, robot_size);

      auto heading = Vec2D::withAngle(angle, sonar_range);
      auto echo_path = Line2D{position, position + heading};
      for (auto const& wall : arena) {
        auto intersection = echo_path.intersection(wall);
        if (intersection.intersects) {
          std::normal_distribution<double> dist(0, sonar_fuzz);
          auto range = intersection.t * sonar_range;
          range += dist(engine);
          m_io_dist[this_sonar] = round(range);

          if (m_io_dist[this_sonar] < m_io_sonalarm_distance &&
              bitmask & m_io_sonarint) {
            m_io_sonalarm_flags = bitmask;
            sendInterrupt(Interrupt::Sonar);
          }

          return;
        }
      }

      // if we didn't find a wall, then there's no echo.
      m_io_dist[this_sonar] = no_echo;
      return;
    }
  }
}

void SComp::updateI2C() {
  // TODO
}

void SComp::updateUART() {
  // TODO
}

void SComp::reset() {
  mw = false;
  pc = 0;
  ac = 0;
  io_write_int = false;
  gie = true;
  iie = 0;
  state = State::FETCH;
  in_hold = false;
  int_req_sync = 0;

  m_io_xpos = 0;
  m_io_ypos = 0;
  m_io_theta = 0;
  m_io_resetpos = 0;

  pos = {};
  heading = 0;
  true_pos = {};
  true_heading = 0;
}

void SComp::fetchAndHandleInt() {
  mw = false;
  ir = mdr;
  io_write_int = false;

  // Handle interrupts if interrupts are enabled
  if (gie && int_req_sync != 0) {
    if (int_req_sync & 0x1) {
      int_ack = 0x1;
      pc = 0x1;
    } else if (int_req_sync & 0x2) {
      int_ack = 0x2;
      pc = 0x2;
    } else if (int_req_sync & 0x4) {
      int_ack = 0x4;
      pc = 0x3;
    } else if (int_req_sync & 0x8) {
      int_ack = 0x8;
      pc = 0x4;
    }

    gie = false;
    ac_saved = ac;
    pc_saved = pc;
    state = State::FETCH;
  } else {
    pc = pc + 1;
    state = State::DECODE;
    int_ack = 0;
  }
}

void SComp::decode() {
  switch (ir >> 11) {
    case 0x0:
      state = State::FETCH;
      break;
    case 0x1:
      state = State::EX_LOAD;
      break;
    case 0x2:
      state = State::EX_STORE;
      break;
    case 0x3:
      state = State::EX_ADD;
      break;
    case 0x4:
      state = State::EX_SUB;
      break;
    case 0x5:
      state = State::EX_JUMP;
      break;
    case 0x6:
      state = State::EX_JNEG;
      break;
    case 0x7:
      state = State::EX_JPOS;
      break;
    case 0x8:
      state = State::EX_JZERO;
      break;
    case 0x9:
      state = State::EX_AND;
      break;
    case 0xA:
      state = State::EX_OR;
      break;
    case 0xB:
      state = State::EX_XOR;
      break;
    case 0xC:
      state = State::EX_SHIFT;
      break;
    case 0xD:
      state = State::EX_ADDI;
      break;
    case 0xE:
      state = State::EX_ILOAD;
      break;
    case 0xF:
      state = State::EX_ISTORE;
      break;
    case 0x10:
      state = State::EX_CALL;
      break;
    case 0x11:
      state = State::EX_RETURN;
      break;
    case 0x12:
      state = State::EX_IN;
      break;
    case 0x13:
      state = State::EX_OUT;
      io_write_int = true;
      break;
    case 0x14: // CLI
      state = State::FETCH;
      iie &= (~ir & 0xF); // AND NOT IR(3 DOWNTO 0)
      break;
    case 0x15: // SEI
      state = State::FETCH;
      iie |= (ir & 0xF); //OR IR(3 DOWNTO 0)
      break;
    case 0x16:
      state = State::EX_RETI;
      break;
    case 0x17:
      state = State::EX_LOADI;
      break;
    default:
      state = State::FETCH;
  }
}

void SComp::load() {
  ac = mdr;
  state = State::FETCH;
}

void SComp::store() {
  mw = true;
  state = State::EX_STORE2;
}

void SComp::store2() {
  mw = false;
  state = State::FETCH;
}

void SComp::add() {
  ac += mdr;
  state = State::FETCH;
}

void SComp::sub() {
  ac -= mdr;
  state = State::FETCH;
}

void SComp::jump() {
  pc = ir & address_mask;
  state = State::FETCH;
}

void SComp::jneg() {
  if (ac < 0) {
    pc = ir & address_mask;
  }

  state = State::FETCH;
}

void SComp::jpos() {
  if (ac > 0) {
    pc = ir & address_mask;
  }

  state = State::FETCH;
}

void SComp::jzero() {
  if (ac == 0) {
    pc = ir & address_mask;
  }

  state = State::FETCH;
}

void SComp::and_() {
  ac &= mdr;
  state = State::FETCH;
}

void SComp::or_() {
  ac |= mdr;
  state = State::FETCH;
}

void SComp::xor_() {
  ac ^= mdr;
  state = State::FETCH;
}

void SComp::shift() {
  // mdr is sign-magnitude 5 bit... because.
  if (mdr &= 0x10) { // negative shift -> right shift
    ac >>= mdr & 0xF;
  } else {
    ac <<= mdr & 0xF;
  }
  
  state = State::FETCH;
}

void SComp::addi() {
  if (ir & 0x400) { // sign bit
    ac += ir | ~address_mask;
  } else {
    ac += ir;
  }
  state = State::FETCH;
}

void SComp::iload() {
  ir &= ~address_mask;
  ir |= (mdr & address_mask);

  state = State::EX_LOAD;
}

void SComp::istore() {
  ir &= ~address_mask;
  ir |= (mdr & address_mask);

  state = State::EX_STORE;
}

void SComp::call() {
  for (size_t i = pc_stack.size()-2; i > 0; --i) {
    pc_stack[i+1] = pc_stack[i];
  }
  pc_stack[0] = pc;
  pc = ir & address_mask;
  state = State::FETCH;
}

void SComp::return_() {
  pc = pc_stack[0];
  for (size_t i = 0; i < pc_stack.size()-1; ++i) {
    pc_stack[i] = pc_stack[i+1];
  }

  state = State::FETCH;
}

void SComp::in() {
  if (in_hold) {
    in_hold = false;
    state = State::FETCH;
  } else {
    ac = io_data;
    in_hold = true;
  }
}

void SComp::out() {
  state = State::EX_OUT2;
}

void SComp::out2() {
  state = State::FETCH;
}

void SComp::loadi() {
  if (ir & 0x400) { // sign bit
    ac = ir | ~address_mask;
  } else {
    ac = ir;
  }
  state = State::FETCH;
}

void SComp::reti() {
  gie = true;
  pc = pc_saved;
  ac = ac_saved;
  state = State::FETCH;
}

void SComp::sendInterrupt(Interrupt i) {
  int bitmask = 1 << (int)i;
  if ((iie & bitmask) && !(int_ack & bitmask)) {
    int_req |= bitmask;
  }
}

double SComp::radiusOfRotation(double leftWheelDisp, double rightWheelDisp) {
  // This is a bit intricate.  We need to determine the center point of rotation
  // based on the speed of both wheels.  For a rough idea of how the shape of
  // this function:
  // - If the numbers are the same then the answer is indeterminate, it could
  //   be either negative or positive infinity.  This must be handled through a
  //   special case.
  // - If the values are the same magnitude but have opposite sign, then the value
  //   of point is zero, representing a rotation about the position.
  // - If one value is zero and the other is non-zero, then the point of rotation
  //   is either one (if the right wheel is motionless) or negative one (if the
  //   left wheel if motionless)

  // special case indeterminate form, return filled infinity

  if (leftWheelDisp == rightWheelDisp) {
    return std::numeric_limits<double>::infinity();
  }

  // otherwise the sign depends on both the slower value and the overall sign of
  // the system
  // read != as xor.
  bool isNeg = (leftWheelDisp < rightWheelDisp) != (leftWheelDisp + rightWheelDisp < 0);

  // because the direction is determined all that matters is the magnitude of
  // the larger vs the smaller value, so let's extract that
  double larger = max(fabs(leftWheelDisp), fabs(rightWheelDisp));
  double smaller = min(fabs(leftWheelDisp), fabs(rightWheelDisp));

  // next let's find the magnitude of the point of rotation
  // this formula comes from considering two concentric circles, we know that
  // the ratio of the circumferences from the larger to smaller.  And we know
  // the distance between the circles, so we can solve for the radius of the
  // larger, then subtract 1 to find the radius at the position.
  double point = axle_track / (1 - copysign(1, leftWheelDisp)*copysign(1, rightWheelDisp)*smaller/larger) - 1;

  // and copy the sign
  if (isNeg) {
    point = -point;
  }

  return point;
}
  
std::pair<Vec2D, double> SComp::circleMotion(Vec2D const& pos, double heading, double leftWheelDisp, double rightWheelDisp) {
  double radius = radiusOfRotation(leftWheelDisp, rightWheelDisp);

  // tdelta is 1/100 of a second
  double radians = (leftWheelDisp + rightWheelDisp) / 2;

  // special case to avoid division by zero, need to handle separately
  if (radius == 0) {
    return {pos, normalizeAngle(heading + rightWheelDisp / M_PI)};
  }

  double angle = normalizeAngle(pos.angle() + M_PI/2);
  double traversedAngle = -radians / radius;
  
  double newAngle = angle + traversedAngle;
  Vec2D positionDelta = Vec2D{cos(newAngle) - cos(angle), sin(newAngle) - sin(angle)} * radius;

  Vec2D position = pos + positionDelta;
  heading = normalizeAngle(newAngle - M_PI/2);

  return {position, heading};
}

}
