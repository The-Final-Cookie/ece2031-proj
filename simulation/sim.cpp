// For ECE2031 project
// Written by Michael Reilly

#include <iostream>
#include <iomanip>
#include "Config.hpp"
#include "MifParse.hpp"
#include "SComp.hpp"

using namespace TFC;

void printUsage(char* programName) {
  cout << "Usage:" << endl;
  cout << programName << " <filename>" << endl;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    printUsage(argv[0]);
    return -1;
  }

  MifParse parser(argv[1]);
  std::vector<int> mifData = parser.parseMif();
  std::vector<bool> isCode = parser.codeSegments();
  int bitwidth = parser.getWidth();

  SComp scomp(mifData, isCode);
  do {
    scomp.stepInstruction();
    if (scomp.getPC() >= 90)
      scomp.setDebug(true);
    else
      scomp.setDebug(false);
  } while(!scomp.getXLeds());

  auto omg = scomp.getMemory();
  
  for (auto entry : mifData) {
    cout << std::setfill('0') << std::setw(std::ceil(bitwidth / 4)) << std::hex << entry;
  }

  cout << endl;
  cout << endl;

  for (auto entry : omg) {
    cout << std::setfill('0') << std::setw(std::ceil(bitwidth / 4)) << std::hex << entry;
  }
  
  return 0;
}
