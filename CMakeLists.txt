project(modbus_test CXX)


add_executable(modbus_test main.cc)
target_compile_features(modbus_test PRIVATE cxx_std_20 c_std_11)