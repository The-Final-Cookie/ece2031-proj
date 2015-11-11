#include <iostream>
#include <iomanip>
#include "Config.hpp"
#include "MifParse.hpp"

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
  int bitwidth = parser.getWidth();

  for (auto entry : mifData) {
    cout << std::setfill('0') << std::setw(std::ceil(bitwidth / 4)) << std::hex << entry;
  }

  return 0;
}
