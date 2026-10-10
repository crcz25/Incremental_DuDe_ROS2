#include "inc_dude/save_path.hpp"

namespace inc_dude {

std::optional<std::string> savePath(const std::string &directory,
                                    const std::string &name,
                                    const std::string &suffix) {
  return directory + "/" + name + suffix;
}

}  // namespace inc_dude
