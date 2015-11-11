#ifndef MIF_PARSE_HPP
#define MIF_PARSE_HPP

#include "Config.hpp"

namespace TFC {

// This code is pretty hacky and manual.  That's ok, I'm not going for some sort of prize or anything.  Not my best work though...

class MifParse {
public:
  MifParse(string const& filename) : filename(filename) {}
  std::vector<int> parseMif();

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
  void readSpaces();
  string readWord();
  int readInt(DataRadix radix);
  DataRadix readRadix();

  void consumeString(string const& toConsume);

  bool consumeAddressEntry();

  string filename;
  stringstream filedata;
  
  size_t bitdepth;
  size_t width;
  DataRadix addressRadix;
  DataRadix dataRadix;

  std::vector<int> result;
};

}

#endif
