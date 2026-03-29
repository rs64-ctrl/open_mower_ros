
# Fetch spdlog if not already available as a target
if(NOT TARGET spdlog::spdlog AND NOT TARGET spdlog)
    Include(FetchContent)

    FetchContent_Declare(
            spdlog
            GIT_REPOSITORY https://github.com/gabime/spdlog
            GIT_TAG v1.14.0
    )

    set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(spdlog)
endif()

# Ensure the alias exists (FetchContent creates 'spdlog' but not always 'spdlog::spdlog')
if(TARGET spdlog AND NOT TARGET spdlog::spdlog)
    add_library(spdlog::spdlog ALIAS spdlog)
endif()
