#pragma once

#include <optional>
#include <string>

namespace inc_dude {

// The path of an image the save trigger writes: <directory>/<name><suffix>,
// or nothing when it cannot be saved there.
std::optional<std::string> savePath(const std::string &directory,
                                    const std::string &name,
                                    const std::string &suffix);

}  // namespace inc_dude
