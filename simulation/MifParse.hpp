#ifndef MIF_PARSE_HPP
#define MIF_PARSE_HPP

#include "ByteArray.hpp"

namespace TFC {

class MifParse {
public:
  MifParse(std::string const& filename) : filename(filename) {}
  ByteArray parseMif();

private:
  enum class DataRadix {
    Invalid = -1,
    Bin,  // Binary
    Hex,  // Hexidecimal
    Dec,  // Signed decimal
    Uns,  // Unsigned decimal
  };

  std::string fileSlurp();

  void readMultilineComment();
  void readSinglelineComment();
  void readSpaces();
  std::string readWord();
  int readInt(DataRadix radix);
  DataRadix readRadix();

  void consumeString(std::string const& toConsume);

  std::string filename;
  std::stringstream filedata;
  
  size_t bitdepth;
  size_t width;
  DataRadix addressRadix;
  DataRadix dataRadix;

};

}

#endif
