protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_SOURCE_DIR} radar.proto)
file(GLOB RADAR_PIS ${CMAKE_CURRENT_SOURCE_DIR}/radar_pi/*.cpp)

add_library(radar_gxhd SHARED f_radar_gxhd.cpp ${PROTO_SRCS} ${CHANS} ${RADAR_PIS})

target_include_directories(radar_gxhd PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(radar_gxhd PUBLIC ${PROJECT_SOURCE_DIR}/channels)
target_include_directories(radar_gxhd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/radar_pi)

install(TARGETS radar_gxhd DESTINATION lib)
