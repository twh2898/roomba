
set(TARGET robbie)

set(HEADER_LIST
    Localizer.hpp
    log.hpp
    MotionControl.hpp
    PID.hpp
    Planning.hpp
    Platform.hpp
    Telemetry.hpp
    World.hpp
    XY.hpp
)
list(TRANSFORM HEADER_LIST PREPEND "include/${TARGET}/")

set(SOURCE_LIST
    main.cpp
    Localizer.cpp
    MotionControl.cpp
    PID.cpp
    Planning.cpp
    Platform.cpp
    Telemetry.cpp
    World.cpp
)
list(TRANSFORM SOURCE_LIST PREPEND "src/")

add_executable(${TARGET}
    ${SOURCE_LIST}
    ${HEADER_LIST}
    include/json.hpp
)

target_include_directories(${TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_property(TARGET ${TARGET} PROPERTY CXX_STANDARD 23)

target_link_libraries(${TARGET}
    spdlog::spdlog
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)

add_custom_command(TARGET ${TARGET} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
    copy ${CMAKE_BINARY_DIR}/controllers/${TARGET}/${TARGET} ${CMAKE_CURRENT_SOURCE_DIR}
)
