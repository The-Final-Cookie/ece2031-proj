#ifndef ENCODE_HPP
#define ENCODE_HPP

#include <string>

namespace TFC {

class ByteArray;

size_t hexEncode(char const* data, size_t len, char* output, size_t outLen = NPos);
size_t hexDecode(char const* src, size_t len, char* output, size_t outLen = NPos);
size_t nibbleDecode(char const* src, size_t len, char* output, size_t outLen = NPos);

size_t base64Encode(char const* data, size_t len, char* output, size_t outLen = NPos);
size_t base64Decode(char const* src, size_t len, char* output, size_t outLen = NPos);

std::string hexEncode(char const* data, size_t len);
std::string base64Encode(char const* data, size_t len);

std::string hexEncode(ByteArray const& data);
ByteArray hexDecode(std::string const& encodedData);

std::string base64Encode(ByteArray const& data);
ByteArray base64Decode(std::string const& encodedData);

}

#endif
