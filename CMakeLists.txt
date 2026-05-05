cmake_minimum_required(VERSION 3.10)

# Project name and language
project(PercivaDemo LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# List source files to compile the executable
add_executable(${CMAKE_PROJECT_NAME}
    main.cpp
    NodeMapInfo.cpp
    FeatureControl.cpp
)

# Path to your OpenCV installation 
set(OpenCV_DIR "C:/opencv/build/x64/vc16/lib") # Adjust this path
set(Spinnaker_DIR "C:/Program Files/Teledyne/Spinnaker") # Adjust this path


# Include external libraries binary files
find_package(OpenCV REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE 
    ${OpenCV_LIBS}
    ${Spinnaker_DIR}/lib64/vs2015/Spinnaker_v140.lib
)   

# Include external libraries header files
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE 
    ${OpenCV_INCLUDE_DIRS}
    ${Spinnaker_DIR}/include
)