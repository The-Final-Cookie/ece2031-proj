#ifndef MIF_PARSE_HPP
#define MIF_PARSE_HPP

#include "ByteArray.hpp"

namespace TFC {

class MifParse {
public:
  MifParse(string const& filename) : filename(filename) {}
  ByteArray parseMif();

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

  Optional<size_t> consumeAddressEntry();

  string filename;
  stringstream filedata;
  
  size_t bitdepth;
  size_t width;
  DataRadix addressRadix;
  DataRadix dataRadix;

};

}

#endif
