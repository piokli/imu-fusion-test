idf_component_register(SRCS "imu-fusion-test.c"
                       INCLUDE_DIRS "."
                       REQUIRES "driver" "lsm6ds33" "i2c_helper" "Fusion")
