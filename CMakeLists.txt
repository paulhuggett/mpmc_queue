project (mpmc CXX)
cmake_minimum_required(VERSION 3.16)

# Warning settings for the major compilers.
set(clang_warnings
    -Weverything
    -Wno-c++98-compat
    -Wno-c++98-compat-pedantic
    -Wno-c++98-compat-bind-to-temporary-copy
    -Wno-exit-time-destructors
    -Wno-global-constructors
    -Wno-padded
)
set(gcc_warnings
    -Wall
    -Wextra
    -pedantic
)
set(msvc_warnings
    /W4
    /wd4324
)

include (FindThreads)

function(target_tweaks name)
    target_compile_features(${name} PUBLIC cxx_std_17)
    target_compile_options(${name} PRIVATE
        $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:${clang_warnings}>
        $<$<CXX_COMPILER_ID:GNU>:${gcc_warnings}>
        $<$<CXX_COMPILER_ID:MSVC>:${msvc_warnings}>
    )
    target_link_libraries(${name} PUBLIC Threads::Threads)
endfunction(target_tweaks)


add_executable(mpmc main.cpp mpmc_queue.hpp)
target_tweaks(mpmc)
add_executable(test test.cpp mpmc_queue.hpp)
target_tweaks(test)
