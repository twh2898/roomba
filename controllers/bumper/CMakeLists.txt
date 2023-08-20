
set(TARGET bumper)

add_executable(${TARGET}
    bumper.cpp
    json.hpp
    Localizer.hpp
    MotionControl.hpp
    PID.hpp
    Planning.hpp
    Roomba.hpp
    simple_cpp_sockets.h
    Telemetry.hpp
)

target_link_libraries(${TARGET}
    spdlog::spdlog
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)

add_custom_command(TARGET ${TARGET} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
    copy ${CMAKE_BINARY_DIR}/controllers/${TARGET}/${TARGET} ${CMAKE_CURRENT_SOURCE_DIR}
)
