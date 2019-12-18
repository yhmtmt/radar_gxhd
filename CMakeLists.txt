add_library(sample SHARED f_sample.cpp)

target_include_directories(sample PUBLIC ${PROJECT_SOURCE_DIR}/include)
install(TARGETS sample DESTINATION lib)
