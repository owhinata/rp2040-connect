# The LSM6DSO driver on Zephyr (CONFIG_LSM6DSO_LOCAL, Kconfig of this directory): a Zephyr
# module (zephyr/module.yml, which the application lists in ZEPHYR_EXTRA_MODULES) built as a
# library of its own from the core of common/drivers/lsm6dso (lsm6dso.c, lsm6dso_cal.c,
# lsm6dso_fifo.c; with CONFIG_SENSOR_ASYNC_API the FIFO stream lsm6dso_stream.c) and this
# binding (lsm6dso_sensor.c; with CONFIG_SENSOR_ASYNC_API the decoder and the RTIO submissions
# lsm6dso_decoder.c, lsm6dso_rtio.c, lsm6dso_rtio_stream.c, and the stream's work queue and INT1
# interrupt lsm6dso_os.c), on ST's lsm6dso_reg.c (hal_st, CONFIG_USE_STDC_LSM6DSO) and Zephyr's
# stmemsc glue (drivers/sensor/st/stmemsc, CONFIG_HAS_STMEMSC).  Its include paths: the
# driver's include/, internal/ and port/, this directory (lsm6dso_os_impl.h,
# lsm6dso_stream_req_impl.h, lsm6dso_instance.h) and stmemsc.h's.  The extended API
# <zephyr/drivers/sensor/lsm6dso.h> (include/zephyr/drivers/sensor/lsm6dso.h here, which reads
# the driver's include/lsm6dso.h) is put on every library's include path, for the application
# and the sensor shell.
if(NOT CONFIG_LSM6DSO_LOCAL)
  return()
endif()

set(lsm6dso_dir ${CMAKE_CURRENT_LIST_DIR}/../..)
cmake_path(NORMAL_PATH lsm6dso_dir)

zephyr_include_directories(include)

zephyr_library_named(lsm6dso_local)
zephyr_library_sources(lsm6dso_sensor.c ${lsm6dso_dir}/lsm6dso.c ${lsm6dso_dir}/lsm6dso_cal.c
  ${lsm6dso_dir}/lsm6dso_fifo.c)
zephyr_library_sources_ifdef(CONFIG_SENSOR_ASYNC_API lsm6dso_decoder.c lsm6dso_rtio.c
  lsm6dso_rtio_stream.c ${lsm6dso_dir}/lsm6dso_stream.c lsm6dso_os.c)
zephyr_library_include_directories(${lsm6dso_dir}/include ${lsm6dso_dir}/internal
  ${lsm6dso_dir}/port ${CMAKE_CURRENT_LIST_DIR} ${ZEPHYR_BASE}/drivers/sensor/st/stmemsc)
