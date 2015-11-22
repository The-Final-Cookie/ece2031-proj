// For ECE2031 project
// Written by Michael Reilly

#ifndef SCOMP_HPP
#define SCOMP_HPP

#include <vector>
#include <array>
#include <random>

#include "Config.hpp"
#include "Line.hpp"

namespace TFC {

class SComp {
public:
  SComp(std::vector<int> const& memory);
  void stepInstruction(size_t count = 1);

  std::vector<int> getMemory() const;

  void setSwitches(uint16_t bitmask);
  uint16_t getRedLeds() const;
  uint16_t getTimer() const;
  uint16_t getXIO() const;
  uint16_t getSevenSeg1() const;
  uint16_t getSevenSeg2() const;
  uint16_t getLCD() const;
  uint16_t getXLeds() const;
  uint16_t getBeep() const;
  uint16_t getCTimer() const;
  uint16_t getUARTData() const;
  uint16_t getUARTRdy() const;

  std::array<uint16_t, 8> getSonarDist() const;
  uint16_t getSonalarmDist() const;
  uint16_t getSonalarmFlags() const;
  uint16_t getSonarInt() const;
  uint16_t getSonarEn() const;

  uint16_t getOdometryLPos() const;
  uint16_t getOdometryLVel() const;
  uint16_t getLVelCmd() const;
  uint16_t getOdometryRPos() const;
  uint16_t getOdometryRVel() const;
  uint16_t getRVelCmd() const;
  uint16_t getOdometryXPos() const;
  uint16_t getOdometryYPos() const;
  uint16_t getOdometryTheta() const;

  Vec2D getTruePos() const;
  double getTrueHeading() const;

  double getTrueLVel() const;
  double getTrueRVel() const;

private:
  enum class State : uint8_t {
		RESET_PC,
		FETCH,
		DECODE,
		EX_LOAD,
		EX_STORE,
		EX_STORE2,
		EX_ADD,
		EX_SUB,
		EX_JUMP,
		EX_JNEG,
		EX_JPOS,
		EX_JZERO,
		EX_AND,
		EX_OR,
		EX_XOR,
		EX_SHIFT,
		EX_ADDI,
		EX_ILOAD,
		EX_ISTORE,
		EX_CALL,
		EX_RETURN,
		EX_IN,
		EX_OUT,
		EX_OUT2,
		EX_LOADI,
		EX_RETI
  };

  enum class IOAddr {
    Switches = 0x00,
    Leds = 0x01,
    Timer = 0x02,
    Xio = 0x03,
    Sseg1 = 0x04,
    Sseg2 = 0x05,
    Lcd = 0x06,
    Xleds = 0x07,
    Beep = 0x0A,
    Ctimer = 0x0C,
    Lpos = 0x80,
    Lvel = 0x82,
    Lvelcmd = 0x83,
    Rpos = 0x88,
    Rvel = 0x8A,
    Rvelcmd = 0x8B,
    I2c_cmd = 0x90,
    I2c_data = 0x91,
    I2c_rdy = 0x92,
    Uart_dat = 0x98,
    Uart_rdy = 0x99,
    Sonar = 0xA0,
    Dist0 = 0xA8,
    Dist1 = 0xA9,
    Dist2 = 0xAA,
    Dist3 = 0xAB,
    Dist4 = 0xAC,
    Dist5 = 0xAD,
    Dist6 = 0xAE,
    Dist7 = 0xAF,
    Sonalarm = 0xB0,
    Sonarint = 0xB1,
    Sonaren = 0xB2,
    Xpos = 0xC0,
    Ypos = 0xC1,
    Theta = 0xC2,
    Resetpos = 0xC3,
    Rin = 0xC8,
    Lin = 0xC9,
  };

  enum class Interrupt : uint8_t {
    Timer = 0x1,
    Sonar = 0x2,
    UART  = 0x4,
    Stall = 0x8,

    SIZE = 4,
  };

  void runState();

  void updateIO();
  void doIoRead();
  void doIoWrite();
  void doIoUpdate();

  void updateTimers();
  void updatePos();
  void updateSonar();
  void updateI2C();
  void updateUART();

  void reset();
  void fetchAndHandleInt();
  void decode();
  void load();
  void store();
  void store2();
  void add();
  void sub();
  void jump();
  void jneg();
  void jpos();
  void jzero();
  void and_();
  void or_();
  void xor_();
  void shift();
  void addi();
  void iload();
  void istore();
  void call();
  void return_();
  void in();
  void out();
  void out2();
  void loadi();
  void reti();

  void sendInterrupt(Interrupt);

  // accepts a normalized vector to represent current position and heading and
  // then how far to turn in radians and how far away the radius is
  std::pair<Vec2D, double> circleMotion(Vec2D const& pos, double heading, double leftWheelDisp, double rightWheelDisp);
  // determines the radius to use from circle motion
  double radiusOfRotation(double lvel, double rvel);

  static constexpr uint16_t address_mask = 0x7FF;
  static constexpr size_t instruction_period = 12500000; // Clock runs at 12.5 MHz

  // Angles are assumed to have a common centerpoint for simplicity.  This is
  // not true.
  static constexpr const std::array<double, 8> sonar_angles =
    {{M_PI/2., M_PI*11./45., M_PI/15., -M_PI/15., -M_PI*11./45., -M_PI/2.,
    -M_PI*4./5., M_PI*4./5.}};

  static constexpr const uint16_t no_echo = 0x7FFF;

  // As much as possible, all distances are specified in mms

  // Sonar distance is tracked in mms and can sense up to 5m away
  static constexpr const uint16_t sonar_range = 5000;
  // but not less than 15cm
  static constexpr const uint16_t sonar_min = 150;

  // Accuracy is only good to +- 1cm though (using it as a stdev at 95%)
  static constexpr const double sonar_fuzz = 5;

  static constexpr const double axle_track = 238;

  // For simplicity, the DE2Bot is modeled as a circle with a radius of its axle
  // track diameter + 50mm for safety, the robot is slightly longer than wide
  static constexpr const double robot_size = axle_track + 50;

  // Arena
  static constexpr const std::array<Line2D, 6> arena = {{
      Line2D{Vec2D(-1524, 1828.8), Vec2D(-304.8, 1828.8)},   // Top left
      Line2D{Vec2D(304.8, 1828.8), Vec2D(1524, 1828.8)},     // Top right
      Line2D{Vec2D(-1524, -1828.8), Vec2D(-1524, 1828.8)},   // Left
      Line2D{Vec2D(1524, -1828.8), Vec2D(1524, 1828.8)},     // Right
      Line2D{Vec2D(-1524, -1828.8), Vec2D(-304.8, -1828.8)}, // Bottom left
      Line2D{Vec2D(304.8, -1828.8), Vec2D(1524, -1828.8)},   // Bottom right
    }};

  static constexpr const double ticks_per_mm = 1.05;

  // This is used to simulate positional drift
  // uncertainty at 0.04% every 1/100 sec higher values are more evil
  // This isn't strictly realistic, as positional drift tends to be correlated with speed
  // and heading drift tends to periodicity with speed when moving forward and
  // tends to be correlated with angular momentum when staying still, but this
  // is a decent approximation.

  static constexpr const double pos_fuzz = 0.0004; // rectal figure
  static constexpr const double heading_fuzz = 0.0004; // rectal figure

  // These constants are MAGIC.  Thanks Kevin!  Units are
  // mm/(centisec*speedUnit) (speedUnit in this case means 0-511 unit used to
  // control the DE2Bot via lvelcmd and rvelcmd)
  // More testing may get better data for this.  I expect the fit to not be
  // strictly linear due to the strange, non-linear nature of rotational
  // friction in a DC motor.
  static constexpr const double speed_slope = 0.0120706637595084;
  static constexpr const double speed_inter = -0.557557985989281;

  // "The acceleration (including deceleration) of each wheel is controlled to
  // 512units/s.  An important side effect of this is that overshoot can be
  // calculated and accounted for.  In general, the distance required to change
  // velocity is (v_1^2 - v_2^2) / 2a , so, simplified and applied to this
  // robot, the distance required to stop can be estimated by Vel^2/1024 (where
  // both VEL and the resulting distance are in robot units).  Extending this to
  // rotation s, assuming in-place rotations with equal but opposite wheel
  // velocities Vel_turn, overshoot (in degrees) can be estimated as Vel_turn^2
  // / 2030."

  // Units are in mm/(sec*centisec), this is directly the approach rate every
  // time odometry updates, both the acceleration and the speed are fuzzed
  static constexpr const double max_accel = 5.376;
  static constexpr const double accel_fuzz = max_accel * 0.04; // rectal figure

  // The standard deviation ranges from about 1.5 to a bit over 2 depending on
  // speed, setting to 2 for safety, this is directly applied to the speed (!)
  // because the percentage uncertainty doesn't seem to be constant, instead
  // there seems to be even more uncertainty in speed at lower speeds in
  // absolute terms
  static constexpr const double speed_stdev = 2.0;

  std::mt19937_64 engine;

  uint64_t executed_instructions;

  std::vector<int> memory;
  uint8_t io_addr;
  uint16_t io_data;
  bool io_write;
  bool io_cycle;

  State state;
  std::array<uint16_t, 10> pc_stack;
  //uint16_t io_in;
  uint16_t ac;
  uint16_t ac_saved;
  uint16_t ir;
  uint16_t mdr;
  uint16_t pc;
  uint16_t pc_saved;
  uint16_t mem_addr;
  bool mw;
  bool io_write_int;
  bool gie;
  uint8_t iie;
  uint8_t int_req;
  uint8_t int_req_sync;
  uint8_t int_ack;
  bool in_hold;

  uint16_t m_io_switches;
  uint16_t m_io_redLeds;
  uint16_t m_io_timer;
  uint16_t m_io_xio;
  uint16_t m_io_sevenSeg1;
  uint16_t m_io_sevenSeg2;
  uint16_t m_io_lcd;
  uint16_t m_io_xleds;
  uint16_t m_io_beep;
  uint16_t m_io_ctimer;
  uint16_t m_io_lpos;
  uint16_t m_io_lvel;
  uint16_t m_io_lvelcmd;
  uint16_t m_io_rpos;
  uint16_t m_io_rvel;
  uint16_t m_io_rvelcmd;
  uint16_t m_io_i2c_cmd;
  uint16_t m_io_i2c_data;
  uint16_t m_io_i2c_rdy;
  uint16_t m_io_uart_dat;
  uint16_t m_io_uart_rdy;
  //uint16_t m_io_sonar; //TODO: what's this?
  std::array<uint16_t, 8> m_io_dist;

  uint16_t m_io_sonalarm_distance;
  uint16_t m_io_sonalarm_flags;

  uint16_t m_io_sonarint;
  uint16_t m_io_sonaren;
  uint16_t m_io_xpos;
  uint16_t m_io_ypos;
  uint16_t m_io_theta;
  uint16_t m_io_resetpos;
  //uint16_t m_io_rin; //TODO: what's this?
  //uint16_t m_io_lin; //TODO: what's this?

  // Used to track the sonars
  uint8_t last_sonar_used;

  // A high precision version of what the bot thinks its position and heading is
  Vec2D pos;
  double heading;

  // And what it actually is
  Vec2D true_pos;
  double true_heading;

  double true_lvel;
  double true_rvel;
};

}

#endif
