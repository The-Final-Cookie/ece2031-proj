// For ECE2031 project
// Written by Michael Reilly

#include "SComp.hpp"

namespace TFC {

SComp::SComp(std::vector<int> const& memory)
: memory(memory), state(State::RESET_PC), pc_stack() {}

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

void SComp::runState() {
  if (mw) {
    memory[mem_addr] = ac;
  } else {
    mdr = memory[mem_addr];
  }

  io_addr = ir & 0xFF;

  mem_addr = (state == State::FETCH) ? pc : ir & addressMask;
  io_cycle = (state == State::EX_IN ||
              state == State::EX_OUT2);

  io_write = io_write_int;

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
    case 0x14:
      state = State::FETCH;
      iie &= (~ir & 0xF); // AND NOT IR(3 DOWNTO 0)
      break;
    case 0x15:
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
  pc = ir & addressMask;
  state = State::FETCH;
}

void SComp::jneg() {
  if (ac < 0) {
    pc = ir & addressMask;
  }

  state = State::FETCH;
}

void SComp::jpos() {
  if (ac > 0) {
    pc = ir & addressMask;
  }

  state = State::FETCH;
}

void SComp::jzero() {
  if (ac == 0) {
    pc = ir & addressMask;
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
    ac += ir | ~addressMask;
  } else {
    ac += ir;
  }
  state = State::FETCH;
}

void SComp::iload() {
  ir &= ~addressMask;
  ir |= (mdr & addressMask);

  state = State::EX_LOAD;
}

void SComp::istore() {
  ir &= ~addressMask;
  ir |= (mdr & addressMask);

  state = State::EX_STORE;
}

void SComp::call() {
  for (size_t i = pc_stack.size()-2; i > 0; --i) {
    pc_stack[i+1] = pc_stack[i];
  }
  pc_stack[0] = pc;
  pc = ir & addressMask;
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
    ac = ir | ~addressMask;
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

}
