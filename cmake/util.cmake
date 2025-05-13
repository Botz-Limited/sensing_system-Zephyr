set(SYS_COMPILE_FLAGS 
   # -Wall
  #  -Wextra
   # -Werror
  #  -Wshadow
   # -Wno-psabi
    CACHE STRING "SYS_COMPILE_FLAGS"
)


function(ProjectTestSetup snippets)
    # Compile commands used by clangd
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON PARENT_SCOPE)
    set(CMAKE_CXX_STANDARD 20 PARENT_SCOPE)

    list(APPEND SNIPPET_ROOT $ENV{APPLICATION_ROOT_PATH})
    set(SNIPPET_ROOT ${SNIPPET_ROOT} PARENT_SCOPE)
    set(SNIPPET ${snippets} PARENT_SCOPE)

    list(APPEND ZEPHYR_EXTRA_MODULES 
        $ENV{APPLICATION_ROOT_PATH}/drivers
    )
    set(ZEPHYR_EXTRA_MODULES ${ZEPHYR_EXTRA_MODULES} PARENT_SCOPE)

    list(APPEND BOARD_ROOT $ENV{APPLICATION_ROOT_PATH})
    set(BOARD_ROOT ${BOARD_ROOT} PARENT_SCOPE)

    list(APPEND DTS_ROOT $ENV{APPLICATION_ROOT_PATH})
    set(DTS_ROOT ${DTS_ROOT} PARENT_SCOPE)
endfunction()

function(SetupTarget target_name is_unit_test)
    set_target_properties(zephyr_interface PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES $<TARGET_PROPERTY:zephyr_interface,INTERFACE_INCLUDE_DIRECTORIES>)
    target_compile_options(${target_name} PRIVATE ${SYS_COMPILE_FLAGS})

    target_include_directories(${target_name} PRIVATE 
        $ENV{APPLICATION_ROOT_PATH}/include
        $ENV{APPLICATION_ROOT_PATH}/drivers
        $ENV{APPLICATION_ROOT_PATH}/src
        $ENV{APPLICATION_ROOT_PATH}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_BINARY_DIR}
    )

    zephyr_include_directories(
        $ENV{APPLICATION_ROOT_PATH}/include
        configuration
        ${ZEPHYR_BASE}/../modules/fs/littlefs
    )

    if(is_unit_test)
        target_compile_options(app PRIVATE -DUNIT_TESTING)
        zephyr_compile_options(-DUNIT_TESTING)
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E rm -f $ENV{APPLICATION_ROOT_PATH}/compile_commands.json
        COMMAND yes | cp -f ${CMAKE_BINARY_DIR}/compile_commands.json $ENV{APPLICATION_ROOT_PATH}/compile_commands.json
        COMMENT "Moving compile_commands.json to $ENV{APPLICATION_ROOT_PATH}"
        VERBATIM
    )
endfunction()
