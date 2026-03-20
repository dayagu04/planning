macro(GenVersionHeaderHere ci_dir)
    message (STATUS "ci_dir=" ${ci_dir})

    execute_process (
    COMMAND git rev-parse --short=8 --vertify HEAD
    OUTPUT_VARIABLE SOURCE_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    )
    message (STATUS "SOURCE_COMMIT(git)=" ${SOURCE_COMMIT})

    configure_file (
    "${ci_dir}/version.h.in"
    "${CMAKE_CURRENT_LIST_DIR}/version.h"
    )
endmacro()
