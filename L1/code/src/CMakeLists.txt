INCLUDE_DIRECTORIES(
${PROJECT_SOURCE_DIR}/include
)

set(LIB_SRC hello.cpp)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

set(APP_SRC useHello.cpp)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

add_library(libhello SHARED ${LIB_SRC})
set_target_properties(libhello PROPERTIES OUTPUT_NAME "sayHello")

add_executable(sayHello ${APP_SRC})
target_link_libraries(sayHello libhello)

