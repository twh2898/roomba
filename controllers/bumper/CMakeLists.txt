
set(TARGET bumper)

add_executable(${TARGET}
    bumper.cpp
    json.hpp
    Loc.hpp
    MC.hpp
    Planning.hpp
    simple_cpp_sockets.h
)

target_link_libraries(${TARGET} ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})

add_custom_command(TARGET ${TARGET} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
    copy ${CMAKE_BINARY_DIR}/controllers/${TARGET}/${TARGET} ${CMAKE_CURRENT_SOURCE_DIR}
)
