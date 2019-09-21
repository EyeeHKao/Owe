project(utils)
add_definitions(-DBUILD_UTILS_UNITTEST=false)

# Set the output directory for the build executables and libraries
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib   )
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/utils )


include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} SHARED src/Timer.cpp
                                   src/Time.cpp
                                   src/Duration.cpp )



#Build test
message(STATUS "Build utils unittest(depend googletest): ${BUILD_UTILS_UNITTEST}")
if(BUILD_UTILS_UNITTEST)
    add_executable(TimerTest test/Timer_unittest.cpp)
    target_link_libraries(TimerTest ${GTEST_LIBRARY} ${GTEST_MAIN_LIBRARY} ${PROJECT_NAME})

    add_executable(TimeTest test/Time_unittest.cpp)
    target_link_libraries(TimeTest ${GTEST_LIBRARY} ${GTEST_MAIN_LIBRARY} ${PROJECT_NAME})
endif()