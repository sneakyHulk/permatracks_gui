project(msg)

add_library(${PROJECT_NAME} src/MagneticFluxDensityDataRawLIS3MDL.cpp src/MagneticFluxDensityDataRawMMC5983MA.cpp src/MagneticFluxDensityDataRawAK09940A.cpp src/MagneticFluxDensityData.cpp src/Message.cpp src/MagneticFluxDensityIntegerData.cpp src/Position.cpp src/Direction.cpp src/DirectionVector.cpp src/Pack.cpp src/Box.cpp src/Array.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)