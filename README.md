# cpp-boilerplate-v2

# C++ Boilerplate v2 Badges
![CICD Workflow status](https://github.com/f-coronado/808x-midterm/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/f-coronado/808x-midterm/branch/main/graph/badge.svg)](https://codecov.io/gh/f-coronado/808x-midterm) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)


## Overview and description

Simple velocity inverse kinematics module. This application can take take the end effector velocity from the user and generate the joint angle velocities needed to achieve the desired end effector velocities. This module assumes the manipulator arm is the franka emika panda arm. First the transformation matrix from the base to end effector is found using the get_transform method from the transform class. The transform matrix is a 4x4 matrix where the upper left 3x3 contains the rotational portion of the transformation and the top right 3x1 is the translational portion of the matrix. The intermediate transform matrices from base to joint 2, 3, 4, etc. are also found. These transformation matrices are needed to construct the jacobian matrix. The jacobian matrix is a 6x6 matrix which is illustrated below: 

![Relative Image](./images/jacobian.png)

In this picture we call the translation portion of the transform matrix from base to end effector x_p. This is needed to calculate the top 3 rows of the jacobian matrix. We take the partial derivative of x_p with respect to each joint. The bottom 3 rows of the jacobian matrix are collected from the 3rd column, top 3 rows of the intermediate transform matrices base to joint 1, joint 2, joint 3, etc. 
The jacobian matrix is needed because the mapping from end effector velocity to joint velocity is achieved by the formula:
q_dot = inv(J) * x_dot
Where J is the jacobian and x_dot is the end effector velocity vector. Proposal.pdf is included for more info.



## Personnel
Fabrizzio Coronado | f-coronado | https://www.linkedin.com/in/fabrizzio-coronado/
<br>I am a 2nd year graduate student at the University of Maryland pursuing a masters in Robotics. After graduating with my Bachelors in Mechanical Engineering in 2021, I started working in the space industry specifically on satellites and would like to pursue a career in space robotics. 




## Standard install via command-line
```bash
# Download the code:
  git https://github.com/f-coronado/808x-midterm.git
  cd 808x-midterm
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
# Run program:
  ./build/app/shell-app
# Run tests:
  cd build/; ctest; cd -
  # or if you have newer cmake
  ctest --test-dir build/
# Build docs:
  cmake --build build/ --target docs
# open a web browser to browse the doc
  open docs/html/index.html
# Clean
  cmake --build build/ --target clean
# Clean and start over:
  rm -rf build/
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


## Links
```bash
# Product backlog, iteration backlog and time log
  https://docs.google.com/spreadsheets/d/1VKGt2zKTjBtlkvJMwKdi0k1ZclaYV6AWt2wLDDGk_b0/edit?usp=sharing

# Quad Chart
  https://docs.google.com/presentation/d/12wDNeGenJ6PZa2YBnvytAr-irsh7FIgMs90facaFUgE/edit?usp=sharing

# Video Explanation
  https://drive.google.com/file/d/1tw8yLYi-4z8CyTvL_b2WaEcO5dOOF1O7/view?usp=sharing
=======
  Phase 0: https://drive.google.com/file/d/1tw8yLYi-4z8CyTvL_b2WaEcO5dOOF1O7/view?usp=sharing

  Phase 1: https://drive.google.com/file/d/1o_8EPgpjgHKsp7_N7xTXA3sdUGlNZmDy/view?usp=sharing
```
## Requirements
```bash
  # make sure you have the eigen library installed
  sudo apt-get install libeigen3-dev
  
