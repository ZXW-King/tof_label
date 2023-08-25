set(YAMLCPP_PREFIX_PATH
        "${YAMLCPP_DIR}/include"
        "${YAMLCPP_DIR}/lib"
        )

list(APPEND CMAKE_PREFIX_PATH "${YAMLCPP_PREFIX_PATH}")

FIND_PATH(YAML_CPP_INCLUDE_DIR NAMES yaml-cpp
        PATHS ${YAMLCPP_DIR}/include
        )
FIND_LIBRARY(YAML_CPP_LIBRARIES
        NAMES ${YAMLCPP_STATIC} yaml-cpp
        PATHS ${YAMLCPP_DIR}/lib
        )

set(YAMLCPP_LIBRARY ${YAML_CPP_LIBRARIES})
message(STATUS "yaml include: " ${YAML_CPP_INCLUDE_DIR})
message(STATUS "yaml lib: " ${YAMLCPP_LIBRARY})
