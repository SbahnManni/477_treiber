cmake_minimum_required(VERSION 3.16)
project(locsim-driver LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Sammle ALLE .cpp unter src/ rekursiv ein
file(GLOB_RECURSE SRC
    CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

# Ein Executable aus allen Quellen
add_executable(locsim_app ${SRC})

# Include-Pfade (damit #include "..." überall klappt)
target_include_directories(locsim_app
    PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/src/config"
)

# Threads (für mutex/condition_variable)
find_package(Threads REQUIRED)
target_link_libraries(locsim_app PRIVATE Threads::Threads)

# Warnungen
if (MSVC)
    target_compile_options(locsim_app PRIVATE /W4)
else()
    target_compile_options(locsim_app PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Ausgabeordner nett
set_target_properties(locsim_app PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

find_package(nlohmann_json 3.0 QUIET)        # optional
target_link_libraries(locsim_app PRIVATE nlohmann_json::nlohmann_json)

if (NOT MSVC)
  target_compile_options(locsim_app PRIVATE -Wno-psabi)
endif()

add_custom_command(TARGET locsim_app POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:locsim_app>/config"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${CMAKE_SOURCE_DIR}/src/config/io_map.json"
          "$<TARGET_FILE_DIR:locsim_app>/config/io_map.json"
)