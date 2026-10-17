# VRPN 7.35's unchanged FindVRPN still checks a legacy FindThreads variable.
# CMake 4 removed it; populate it only after the actual compiler/linker probe.
if(CMAKE_VERSION VERSION_GREATER_EQUAL 4.0)
  find_package(Threads REQUIRED)
  set(CMAKE_HAVE_THREADS_LIBRARY "${Threads_FOUND}" CACHE BOOL "Legacy VRPN thread discovery compatibility" FORCE)
endif()
