cmake_minimum_required(VERSION 3.0)
project(testing_gtsam)


set(SOURCE "src/main.cpp")
set(TEST_SOURCE "test/main.cpp")

file(GLOB CLASS_SOURCES
     "classes/*.cpp"
     "include/*.h")

include_directories(${PROJECT_SOURCE_DIR}/include)

#find gtsam library
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(Boost 1.50 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})


add_executable(application
               ${SOURCE}
               ${CLASS_SOURCES})

add_executable(tests
               ${TEST_SOURCE}
               ${CLASS_SOURCES})

               
target_link_libraries(application
                      gtsam
                      Eigen3::Eigen)



target_link_libraries(tests
                      gtsam
                      Eigen3::Eigen)
