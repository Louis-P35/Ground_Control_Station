cmake_minimum_required(VERSION 3.16)
project(GroundControlStation VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Network OpenGL OpenGLWidgets Multimedia MultimediaWidgets)

set(SOURCES
    src/main.cpp

    src/backend/AppLogger.cpp
    src/backend/UdpLink.cpp
    src/backend/PacketParser.cpp
    src/backend/TelemetryState.cpp
    src/backend/CommandSender.cpp

    src/ui/MainWindow.cpp
    src/ui/widgets/DroneWidget3D.cpp
    src/ui/widgets/TrackingWidget3D.cpp
    src/ui/widgets/CompassWidget.cpp
    src/ui/widgets/JoystickWidget.cpp
    src/ui/widgets/Mtf01Widget.cpp
    src/ui/widgets/GpsWidget.cpp
    src/ui/widgets/MotorWidget.cpp
    src/ui/widgets/StatusWidget.cpp
    src/ui/widgets/MapWidget.cpp
    src/ui/widgets/VideoWidget.cpp
    src/ui/widgets/PidConfigWidget.cpp
    src/ui/widgets/BarometerWidget.cpp
    src/ui/widgets/CalibrationWidget.cpp
    src/ui/widgets/GraphWidget.cpp
    src/ui/widgets/TerminalWidget.cpp
    src/ui/widgets/FftWidget.cpp
)

set(HEADERS
    src/backend/AppLogger.h
    src/backend/UdpLink.h
    src/backend/PacketParser.h
    src/backend/TelemetryState.h
    src/backend/CommandSender.h

    src/ui/MainWindow.h
    src/ui/widgets/DroneWidget3D.h
    src/ui/widgets/TrackingWidget3D.h
    src/ui/widgets/CompassWidget.h
    src/ui/widgets/JoystickWidget.h
    src/ui/widgets/Mtf01Widget.h
    src/ui/widgets/GpsWidget.h
    src/ui/widgets/MotorWidget.h
    src/ui/widgets/StatusWidget.h
    src/ui/widgets/MapWidget.h
    src/ui/widgets/VideoWidget.h
    src/ui/widgets/PidConfigWidget.h
    src/ui/widgets/BarometerWidget.h
    src/ui/widgets/CalibrationWidget.h
    src/ui/widgets/GraphWidget.h
    src/ui/widgets/TerminalWidget.h
    src/ui/widgets/FftWidget.h
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE src common)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Network
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::Multimedia
    Qt6::MultimediaWidgets
)

# Fixed-function OpenGL (glBegin/glEnd etc.) requires opengl32 on Windows
if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE opengl32)
endif()

if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
endif()

# Unit tests — build and register with CTest.
# Run with: cmake --build build --target GCSTests && ctest --test-dir build -V
enable_testing()
add_subdirectory(tests)
