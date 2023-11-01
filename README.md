# 808x-midterm

![CICD Workflow status](https://github.com/f-coronado/808x-midterm/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg?branch=iteration2)
[![codecov](https://codecov.io/gh/f-coronado/808x-midterm/branch/iteration2/graph/badge.svg)](https://codecov.io/gh/f-coronado/808x-midterm)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## Overview and description

This is a simple velocity inverse kinematics module. This application can take take the end effector velocity from the user and generate the joint angle velocities needed to achieve the desired end effector velocities. This module assumes the manipulator arm is the franka emika panda arm. In order to calculate inverse kinematics, the following equation is used: 

q_dot = inv(J) * x_dot

Where x_dot is the end effector velocity, q_dot are the joint angle velocities that we are solving for. X_dot is the user defined end effector velocities but J is dependent on the manipulator used. For easability, the jacobian matrix was found in resources/jacobian.py using symbolic libraries and forward kinematics then coded into the jacobian class.
The jacobian matrix is needed because the mapping from end effector velocity to joint velocity is achieved using the formula mentioned earlier. Where J is the jacobian and x_dot is the end effector velocity vector. Proposal.pdf is included for more info on the plan for creating this module.


## Personnel
Fabrizzio Coronado | f-coronado | https://www.linkedin.com/in/fabrizzio-coronado/
<br>I am a 2nd year graduate student at the University of Maryland pursuing a masters in Robotics. After graduating with my Bachelors in Mechanical Engineering in 2021, I started working in the space industry specifically on satellites and would like to pursue a career in space robotics. 

## Install via command-line
```bash
# Download the code:
  git https://github.com/f-coronado/808x-midterm.git
  cd 808x-midterm
```
## How to build the project
```bash
# Configure the project and generate a native build system:
# Must re-run this command whenever any CMakeLists.txt file has been changed.
  cmake -S ./ -B build/
# Compile and build the project:
# rebuild only files that are modified since the last build
  cmake --build build/
  # or rebuild everything from scracth
  cmake --build build/ --clean-first
  # to see verbose output, do:
  cmake --build build/ --verbose
```
## Demo, Tests, Documentation, Bugs

```bash
# To view the demo, run the program:
  ./build/app/shell-app
# To run the tests, enter:
  cd build/; ctest; cd -
  # or if you have newer cmake
  ctest --test-dir build/
# Clean
  cmake --build build/ --target clean
# Clean and start over:
  rm -rf build/
# To generate doxygen documentation:
  cmake --build build/ --target docs
# open a web browser to browse the doc
  open docs/html/index.html
# No actual bugs were found, only warnings. See below
  open results/cppcheck.txt
  # Or you can run it yourself in root
  cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" )
```
## Creating coverage reports
```bash
# if you don't have gcovr or lcov installed, do:
  sudo apt-get install gcovr lcov
# Set the build type to Debug and WANT_COVERAGE=ON
  cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -S ./ -B build/
# Now, do a clean compile, run unit test, and generate the covereage report
  cmake --build build/ --clean-first --target all test_coverage
# open a web browser to browse the test coverage report
  open build/test_coverage/index.html

This generates a index.html page in the build/test_coverage sub-directory that can be viewed locally in a web browser.

You can also get code coverage report for the shell-app target, instead of unit test. Repeat the previous 2 steps but with the app_coverage target:

# Now, do another clean compile, run shell-app, and generate its covereage report
  cmake --build build/ --clean-first --target all app_coverage
# open a web browser to browse the test coverage report
  open build/app_coverage/index.html

This generates a index.html page in the build/app_coverage sub-directory that can be viewed locally in a web browser.
```

## Links to AIP process
```bash
# All files related the AIP process can be found below
  https://drive.google.com/drive/folders/16f_gyT_-XUVV44Xeudwd2JyG00QdQZ-g?usp=sharing

# Product backlog, iteration backlog and time log
  https://docs.google.com/spreadsheets/d/1K1HQfK7hzTwkgw_q-7HrA40nc2Vym68gRzik8PI6Igc/edit?usp=sharing  

# Planning Meetings folder
  https://drive.google.com/drive/folders/1IQSOq1343M5uGwHwAa6Bk0-hjqPUniUA?usp=sharing

# Quad Chart
  https://docs.google.com/presentation/d/12wDNeGenJ6PZa2YBnvytAr-irsh7FIgMs90facaFUgE/edit?usp=sharing

# Phase Video Updates  
  Phase 0: https://drive.google.com/file/d/1tw8yLYi-4z8CyTvL_b2WaEcO5dOOF1O7/view?usp=sharing

  Phase 1: https://drive.google.com/file/d/1o_8EPgpjgHKsp7_N7xTXA3sdUGlNZmDy/view?usp=sharing


```
## Requirements
```bash
# make sure you have the eigen library installed
  sudo apt-get install libeigen3-dev

# if you want to check for bugs
  sudo apt install cppcheck
  # after installing, run the following in the root directory
  cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" )
```

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
