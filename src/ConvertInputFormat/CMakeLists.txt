cmake_minimum_required(VERSION 3.5.1)
project(ConvertInputFormat)

if(APPLE OR UNIX)
  add_executable(ConvertInputFormat main.cpp)
else() # windows
  add_executable(ConvertInputFormat main.cpp) #  "${CMAKE_CURRENT_BINARY_DIR}/energyplus.rc" )
endif()

# Detect OpenMP support in a compiler. If the compiler supports OpenMP, the
# flags to compile with OpenMP are returned and added.
find_package(OpenMP)
if(OPENMP_FOUND)
  target_link_libraries(ConvertInputFormat PRIVATE OpenMP::OpenMP_CXX)
endif()

target_link_libraries(ConvertInputFormat PRIVATE energyplusparser project_options project_warnings)

set_target_properties(ConvertInputFormat PROPERTIES VERSION ${ENERGYPLUS_VERSION})
set_target_properties(ConvertInputFormat PROPERTIES FOLDER Auxiliary)

install(TARGETS ConvertInputFormat DESTINATION ./)
