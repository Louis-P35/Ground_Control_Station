find_package(Qt6 REQUIRED COMPONENTS Test Core Network)

set(TEST_SOURCES
    main.cpp
    TestHelpers.h
    MockUdpLink.h
    TestPacketParser.cpp
    TestCommandSender.cpp
    TestTelemetryState.cpp
    # Backend sources compiled directly into the test binary.
    # This avoids depending on the main executable target (which pulls in all widgets).
    ${CMAKE_SOURCE_DIR}/src/backend/AppLogger.cpp
    ${CMAKE_SOURCE_DIR}/src/backend/PacketParser.cpp
    ${CMAKE_SOURCE_DIR}/src/backend/TelemetryState.cpp
    ${CMAKE_SOURCE_DIR}/src/backend/UdpLink.cpp
    ${CMAKE_SOURCE_DIR}/src/backend/CommandSender.cpp
)

add_executable(GCSTests ${TEST_SOURCES})

target_include_directories(GCSTests PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(GCSTests PRIVATE
    Qt6::Test
    Qt6::Core
    Qt6::Network
)

add_test(NAME GCSTests COMMAND GCSTests)
