#pragma once

#include <cstddef>
#include <optional>
#include <string>

namespace inc_dude {

// The longest name a saved image may have (without its suffix).
inline constexpr std::size_t kMaxSaveNameLength = 128;

// Whether `name` may name a saved image: 1 to kMaxSaveNameLength characters,
// only letters, digits, '-', '_' and '.', and neither "." nor any "..".
// Anything else (separators, NUL, absolute paths) is rejected.
bool isValidSaveName(const std::string &name);

// The path of an image the save trigger writes: <directory>/<name><suffix>,
// or nothing when it cannot be saved there: `name` is not valid, the path
// (with symbolic links resolved) is not directly in `directory`, or the
// path is itself a symbolic link (even a dangling one).
std::optional<std::string> savePath(const std::string &directory,
                                    const std::string &name,
                                    const std::string &suffix);

}  // namespace inc_dude
