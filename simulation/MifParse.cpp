#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>

#include "MifParse.hpp"

namespace TFC {

string MifParse::fileSlurp() {
  string res;
  stringstream resStream(res);
  ifstream file(filename);

  if (!file.is_open()) {
    throw std::exception();
  }

  string line;

  while (file.good()) {
    getline(file, line);
    resStream << line;
  }

  file.close();

  return res;
}

// Each of the following functions leave the iterator past the thing it is
// supposed to have read.  Leaving the caller ready to call the next thing.
// The caller must check for eof where not explicitly prohibited.

void MifParse::readMultilineComment() {
  char startComment = filedata.get();
  assert(startComment == '%');
  _unused(startComment);
  filedata.ignore(std::numeric_limits<std::streamsize>::max(), '%');

  if (!filedata.good())
    throw std::exception();
}

void MifParse::readSinglelineComment() {
  char startComment = filedata.get();
  assert(startComment == '-');
  startComment = filedata.get();
  assert(startComment == '-');
  _unused(startComment);

  filedata.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void MifParse::readSpaces() {
  std::locale loc;
  while (filedata.good()) {
    char next = filedata.peek();
    if (isspace(next, loc)) {
      filedata.get();
    } else if (next == '-') {
      readSinglelineComment();
    } else if (next == '%') {
      readMultilineComment();
    } else {
      return;
    }
  }
}

string MifParse::readWord() {
  readSpaces();

  string res;
  stringstream ss(res);

  while (filedata.good()) {
    char next = filedata.peek();
    if (isalnum(next)) {
      ss << filedata.get();
    } else {
      break;
    }
  }

  return res;
}

void MifParse::consumeString(string const& toConsume) {
  for (char consume : toConsume) {
    if (consume != filedata.get())
      throw std::exception();
  }
}

int MifParse::readInt(DataRadix radix) {
  int res;
  switch (radix) {
    case DataRadix::Bin:
      {
        std::bitset<sizeof(int)> temp(filedata.str(), 0, width);
        res = temp.to_ulong();
      }
      break;
    case DataRadix::Hex:
      filedata >> std::hex >> res;
      break;
    case DataRadix::Dec:
      filedata >> std::dec >> res;
      while (res < 0) {
        res += 1 << width; // this might be problematic if we're doing large
                           // bitdepths.  But for the purposes of this
                           // project it's just fine.
      }
      break;
    case DataRadix::Uns:
      filedata >> std::dec >> res;
      if (res < 0) {
        throw std::exception(); // yeah, no negatives allows in unsigned
      }
      break;
    default:
      throw std::exception();
  }

  return res;
}

MifParse::DataRadix MifParse::readRadix() {
  char type = filedata.get();
  switch (type) {
    case 'B':
      consumeString("IN");
      return DataRadix::Bin;

    case 'H':
      consumeString("EX");
      return DataRadix::Bin;

    case 'D':
      consumeString("EC");
      return DataRadix::Bin;

    case 'U':
      consumeString("NS");
      return DataRadix::Bin;

    default:
      throw std::exception();
  }
}

ByteArray MifParse::parseMif() {
  ByteArray res;

  filedata = stringstream(fileSlurp());

  bitdepth = 0;
  width = 0;
  addressRadix = DataRadix::Invalid;
  dataRadix = DataRadix::Invalid;

  while (filedata.good()) {
    string token = readWord();

    consumeString("=");
    if (token == "DEPTH") {
      bitdepth = readInt(DataRadix::Uns);
    } else if (token == "WIDTH") {
      width = readInt(DataRadix::Uns);
    } else if (token == "ADDRESS_RADIX") {
      addressRadix = readRadix();
    } else if (token == "DATA_RADIX") {
      dataRadix = readRadix();
    } else if (token == "CONTENT") {
      consumeString("BEGIN");

      res.fill(bitdepth*width, '\0');

      size_t consumedWords = 0;

      while (consumedWords < bitdepth) {
        
      }

      consumeString("END;");
      break; // content is always last
    } else {
      throw std::exception();
    }
    
    consumeString(";");
  }

  return res;
}

}
