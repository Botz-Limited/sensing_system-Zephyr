# Shell commands module

if(CONFIG_SHELL)
    target_sources_ifdef(CONFIG_LOG_DECODER_SHELL_CMD app PRIVATE
        log_decoder_cmd.cpp
    )
endif()