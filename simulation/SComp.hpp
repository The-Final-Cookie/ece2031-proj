// For ECE2031 project
// Written by Michael Reilly

#ifndef SCOMP_HPP
#define SCOMP_HPP

#include "Config.hpp"
#include <vector>
#include <array>

namespace TFC {

class SComp {
public:
  SComp(std::vector<int> const& memory);
  void stepInstruction(size_t count = 1);

  std::vector<int> getMemory() const;

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

  void runState();

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

  static constexpr uint16_t addressMask = 0x7FF;

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

};

}

#endif
