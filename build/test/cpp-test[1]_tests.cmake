add_test( jacobianTest.getJ /home/fabrizzio/Downloads/Grad_School/808X/midterm/v2/808x-midterm/build/test/cpp-test [==[--gtest_filter=jacobianTest.getJ]==] --gtest_also_run_disabled_tests)
set_tests_properties( jacobianTest.getJ PROPERTIES WORKING_DIRECTORY /home/fabrizzio/Downloads/Grad_School/808X/midterm/v2/808x-midterm/build/test)
add_test( jointAnglesTest.getQdot /home/fabrizzio/Downloads/Grad_School/808X/midterm/v2/808x-midterm/build/test/cpp-test [==[--gtest_filter=jointAnglesTest.getQdot]==] --gtest_also_run_disabled_tests)
set_tests_properties( jointAnglesTest.getQdot PROPERTIES WORKING_DIRECTORY /home/fabrizzio/Downloads/Grad_School/808X/midterm/v2/808x-midterm/build/test)
set( cpp-test_TESTS jacobianTest.getJ jointAnglesTest.getQdot)
