
set(TARGET robbie)

set(UTIL_TARGET util)

set(UTIL_HEADER_LIST
    Config.hpp
    log.hpp
    PID.hpp
    Telemetry.hpp
    XY.hpp
)
list(TRANSFORM UTIL_HEADER_LIST PREPEND "include/${TARGET}/${UTIL_TARGET}/")

set(UTIL_SOURCE_LIST
    Config.cpp
    log.cpp
    PID.cpp
    Telemetry.cpp
)
list(TRANSFORM UTIL_SOURCE_LIST PREPEND "src/${UTIL_TARGET}/")

add_library(${UTIL_TARGET} OBJECT ${UTIL_SOURCE_LIST} ${UTIL_HEADER_LIST})

target_include_directories(${UTIL_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(BASE_TARGET base)

set(BASE_HEADER_LIST
    Localizer.hpp
    MotionControl.hpp
    Platform.hpp
    Robbie.hpp
)
list(TRANSFORM BASE_HEADER_LIST PREPEND "include/${TARGET}/${BASE_TARGET}/")

set(BASE_SOURCE_LIST
    Localizer.cpp
    MotionControl.cpp
    Platform.cpp
    Robbie.cpp
)
list(TRANSFORM BASE_SOURCE_LIST PREPEND "src/${BASE_TARGET}/")

add_library(${BASE_TARGET} OBJECT ${BASE_SOURCE_LIST} ${BASE_HEADER_LIST})

target_link_libraries(${BASE_TARGET}
    $<TARGET_OBJECTS:util>
)

target_include_directories(${BASE_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# #######

set(HEADER_LIST
    Planning.hpp
    World.hpp
)
list(TRANSFORM HEADER_LIST PREPEND "include/${TARGET}/")

set(SOURCE_LIST
    main.cpp
    Planning.cpp
    World.cpp
)
list(TRANSFORM SOURCE_LIST PREPEND "src/")

add_executable(${TARGET}
    ${SOURCE_LIST}
    ${HEADER_LIST}
    include/json.hpp
)

target_include_directories(${TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_property(TARGET ${TARGET} PROPERTY CXX_STANDARD 17)

target_link_libraries(${TARGET}
    $<TARGET_OBJECTS:${UTIL_TARGET}>
    $<TARGET_OBJECTS:${BASE_TARGET}>
    spdlog::spdlog
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)

add_custom_command(TARGET ${TARGET} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
    copy ${CMAKE_BINARY_DIR}/controllers/${TARGET}/${TARGET} ${CMAKE_CURRENT_SOURCE_DIR}
)
