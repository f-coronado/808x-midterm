# Any C++ source files needed to build this target (shell-app).
find_package(Eigen3 REQUIRED)
add_executable(shell-app
  # list of source cpp files:
  main.cpp
  )

# Any include directories needed to build this target.
# Note: we do not need to specify the include directories for the
# dependent libraries, they are automatically included.
target_include_directories(shell-app PUBLIC
  # list include directories:
  ${CMAKE_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  )

# Any dependent libraires needed to build this target.
target_link_libraries(shell-app PRIVATE
  # list of libraries
  myLib2
  myLib1
  )

target_link_options(shell-app PUBLIC
  --static
  )