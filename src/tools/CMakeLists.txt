# Log decoder tool module

if(CONFIG_LOG_DECODER_TOOL)
    target_sources(app PRIVATE
        log_decoder_app.cpp
    )
endif()