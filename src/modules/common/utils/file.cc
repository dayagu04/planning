#include "file.h"

namespace planning {
namespace common {
namespace util {

std::string ReadFile(const std::string &path) {
  FILE *file = fopen(path.c_str(), "r");
  assert(file != nullptr);
  std::shared_ptr<FILE> fp(file, [](FILE *file) { fclose(file); });
  if (fp == nullptr){
    return "";
  }
  fseek(fp.get(), 0, SEEK_END);
  std::vector<char> content(ftell(fp.get()));
  fseek(fp.get(), 0, SEEK_SET);
  auto read_bytes = fread(content.data(), 1, content.size(), fp.get());
  assert(read_bytes == content.size());
  (void)read_bytes;
  return std::string(content.begin(), content.end());
}

}  // namespace util
}  // namespace common
}  // namespace planning