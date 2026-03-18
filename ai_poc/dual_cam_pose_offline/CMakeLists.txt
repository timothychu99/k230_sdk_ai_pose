set(src
    main.cc
    ov5647_calibration.c
    pixel_to_ray_calib.c
    ../pose_detect/utils.cc
    ../pose_detect/ai_base.cc
    ../pose_detect/pose_detect.cc
)
set(bin dual_cam_pose_offline.elf)

set(reader_src keypoint_fifo_reader.cc triangulation.cpp)
set(reader_bin keypoint_fifo_reader.elf)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../pose_detect)
include_directories(${nncase_sdk_root}/riscv64/rvvlib/include)
include_directories(${k230_sdk}/src/big/mpp/userapps/api/)
include_directories(${k230_sdk}/src/big/mpp/include)
include_directories(${k230_sdk}/src/big/mpp/include/comm)
include_directories(${k230_sdk}/src/big/mpp/userapps/sample/sample_vo)
include_directories(${k230_sdk}/src/common/cdk/user/component/datafifo/include)
link_directories(${k230_sdk}/src/common/cdk/user/component/datafifo/host/lib)
link_directories(${nncase_sdk_root}/riscv64/rvvlib/)

# Main app: built with musl (default toolchain). Skip when building reader-only with glibc.
if(NOT BUILD_KEYPOINT_READER_GLIBC)
  add_executable(${bin} ${src})
  target_link_libraries(${bin} -Wl,--start-group rvv Nncase.Runtime.Native nncase.rt_modules.k230 functional_k230 sys vicap vb cam_device cam_engine
   hal oslayer ebase fpga isp_drv binder auto_ctrol common cam_caldb isi 3a buffer_management cameric_drv video_in virtual_hal start_engine cmd_buffer
   switch cameric_reg_drv t_database_c t_mxml_c t_json_c t_common_c vo connector sensor atomic dma datafifo -Wl,--end-group)

  target_link_libraries(${bin} opencv_imgcodecs opencv_imgproc opencv_core zlib libjpeg-turbo libopenjp2 libpng libtiff libwebp csi_cv)
  install(TARGETS ${bin} DESTINATION bin)
endif()

# Keypoint FIFO reader: for little core Linux, built with Linux (glibc) toolchain only.
# Invoked from build_app_sub.sh with -DCMAKE_TOOLCHAIN_FILE=../cmake/Riscv64_glibc.cmake
# and -DBUILD_KEYPOINT_READER_GLIBC=ON. Uses default linker script (no -T link.lds) so the
# binary loads at a normal Linux address. Pattern from working hello_linux.
if(BUILD_KEYPOINT_READER_GLIBC)
  add_executable(${reader_bin} ${reader_src})
  target_compile_features(${reader_bin} PRIVATE cxx_std_17)
  set_target_properties(${reader_bin} PROPERTIES LINKER_LANGUAGE CXX)
  # Little core Linux: same arch/abi as hello_linux, no custom link.lds
  target_compile_options(${reader_bin} PRIVATE -march=rv64imafdc -mabi=lp64d)
  target_link_options(${reader_bin} PRIVATE -march=rv64imafdc -mabi=lp64d -static)
  # Reader runs on little core: use datafifo slave lib (reader side); pixel_to_ray_calib = no OpenCV
  target_include_directories(${reader_bin} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${k230_sdk}/src/big/mpp/include
    ${k230_sdk}/src/common/cdk/user/component/datafifo/include
  )
  target_link_directories(${reader_bin} PRIVATE
    ${k230_sdk}/src/big/mpp/userapps/lib
  )
  target_link_libraries(${reader_bin}
    -Wl,--start-group
    oslayer
    ebase
    ${k230_sdk}/src/common/cdk/user/component/datafifo/slave/lib/libdatafifo.a
    sys
    -Wl,--end-group
    pthread
    m
  )
  install(TARGETS ${reader_bin} DESTINATION bin)
endif()

