#include "inc_dude/save_path.hpp"

#include <filesystem>
#include <system_error>

namespace inc_dude {

namespace {

bool allowed(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
         (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.';
}

}  // namespace

bool isValidSaveName(const std::string &name) {
  if (name.empty() || name.size() > kMaxSaveNameLength || name == "." ||
      name.find("..") != std::string::npos) {
    return false;
  }
  for (const char c : name) {
    if (!allowed(c)) {
      return false;  // also '/', '\\', NUL and anything outside ASCII
    }
  }
  return true;
}

std::optional<std::string> savePath(const std::string &directory,
                                    const std::string &name,
                                    const std::string &suffix) {
  if (!isValidSaveName(name)) {
    return std::nullopt;
  }
  namespace fs = std::filesystem;
  std::error_code error;
  const fs::path base = fs::weakly_canonical(fs::path(directory), error);
  if (error) {
    return std::nullopt;
  }
  const fs::path path = fs::weakly_canonical(base / (name + suffix), error);
  if (error || path.parent_path() != base) {
    return std::nullopt;  // e.g. a link in the directory that points out
  }
  // A dangling link is not resolved above, but writing follows it.
  if (fs::is_symlink(fs::symlink_status(path, error))) {
    return std::nullopt;
  }
  return path.string();
}

}  // namespace inc_dude
