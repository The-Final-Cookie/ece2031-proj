// For ECE2031 project
// Written by Michael Reilly

#ifndef MIF_PARSE_HPP
#define MIF_PARSE_HPP

#include "Config.hpp"
#include <sstream>
#include <vector>

namespace TFC {

// This code is pretty hacky and manual.  That's ok, I'm not going for some sort of prize or anything.  Not my best work though...

class MifParse {
public:
  MifParse(string const& filename) : filename(filename) {}
  std::vector<int> parseMif();
  size_t getWidth() const;

  // Same length as parseMif return, marks each word as coming from ASM commands
  // or DW commands.  If DW, false (data), otherwise true (code).  This is used
  // in diagnostics to make sure that code isn't read from and code isn't
  // written to.
  std::vector<bool> codeSegments() const;

private:
  enum class DataRadix {
    Invalid = -1,
    Bin,  // Binary
    Hex,  // Hexidecimal
    Dec,  // Signed decimal
    Uns,  // Unsigned decimal
  };

  string fileSlurp();

  void readMultilineComment();
  void readSinglelineComment();
  void readSinglelineCommentWithData(size_t offsetStart, size_t offsetEnd);
  void readSpaces();
  void readSpacesWithData(size_t offsetStart, size_t offsetEnd);
  string readWord();
  int readInt(DataRadix radix);
  DataRadix readRadix();

  void consumeString(string const& toConsume);
  bool tryConsumeString(string const& toConsume);

  bool consumeAddressEntry();

  string filename;
  stringstream filedata;
  
  size_t bitdepth;
  size_t width;
  DataRadix addressRadix;
  DataRadix dataRadix;

  std::vector<int> result;
  std::vector<bool> isCode;
};

}

#endif
