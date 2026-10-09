# Copies a compile_commands.json without the entries whose file matches
# EXCLUDE (a regex), keeping one entry per file, for ament_clang_tidy, which
# lints every entry and has no exclude option.
#
#   cmake -DINPUT=<db> -DOUTPUT=<db> -DEXCLUDE=<regex> -P filter_compile_commands.cmake

# string(JSON) needs CMake 3.19.
cmake_minimum_required(VERSION 3.19)

file(READ "${INPUT}" db)
string(JSON count LENGTH "${db}")
set(out "[")
set(separator "")
set(seen "")
if(count GREATER 0)
  math(EXPR last "${count} - 1")
  foreach(i RANGE ${last})
    string(JSON file GET "${db}" ${i} file)
    if(NOT file MATCHES "${EXCLUDE}" AND NOT file IN_LIST seen)
      list(APPEND seen "${file}")
      string(JSON entry GET "${db}" ${i})
      string(APPEND out "${separator}\n${entry}")
      set(separator ",")
    endif()
  endforeach()
endif()
string(APPEND out "\n]\n")
file(WRITE "${OUTPUT}" "${out}")
