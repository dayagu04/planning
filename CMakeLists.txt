cmake_minimum_required (VERSION 2.8)

project(planning)

find_package(Protobuf REQUIRED)
find_package(Gflags REQUIRED)

include_directories(.)
## MACRO: 遍历文件夹并添加到SRC_LIST中
MACRO(SUBDIRLIST_SRC DIR SRC_LIST)
  IF(IS_DIRECTORY ${DIR})
    message("The DIRs are :")
    message(${DIR})
    include_directories(${DIR})
    FILE(GLOB SRC_TEMP ${DIR}/*.cc ${DIR}/*.cpp)
    FOREACH(SRCS_SUB ${SRC_TEMP})
      LIST(APPEND SRC_LIST_ ${SRCS_SUB})
    ENDFOREACH()
    FILE(GLOB DIRS ${DIR}/*)
    FOREACH(SUB ${DIRS})
      SUBDIRLIST_SRC(${SUB} SRC_LIST_)
    ENDFOREACH()
  ENDIF()
  SET(${SRC_LIST} ${SRC_LIST_})
ENDMACRO()

add_subdirectory(src)
