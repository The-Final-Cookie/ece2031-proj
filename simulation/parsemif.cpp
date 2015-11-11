#include <iostream>
#include "Config.hpp"

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

  

  return 0;
}
