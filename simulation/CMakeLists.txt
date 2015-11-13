PROJECT (tfc)

CMAKE_MINIMUM_REQUIRED (VERSION 3.0)
SET (CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

SET (CMAKE_CONFIGURATION_TYPES Debug RelWithAsserts RelWithDebInfo Release)
# CMake requires this to be set for every custom confuguration for some reason
SET (CMAKE_EXE_LINKER_FLAGS_RELWITHASSERTS "" CACHE STRING "" FORCE)
# Update the docstring on CMAKE_BUILD_TYPE to show what options we actually
# allow
SET (CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING "Choose the type of build, options are: Debug RelWithAsserts RelWithDebInfo Release" FORCE)

INCLUDE (TargetArch)
TARGET_ARCHITECTURE (CMAKE_TARGET_ARCHITECTURE)

INCLUDE (GetGitRevisionDescription)
GET_GIT_HEAD_REVISION (TFC_GIT_REFSPEC TFC_GIT_HASHVAR)
SET (TFC_SOURCE_REVISION "${TFC_GIT_HASHVAR}")

ENABLE_TESTING()

IF (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  SET (CMAKE_COMPILER_IS_CLANGXX 1)
ENDIF ()

IF (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  SET (LINUX 1)
ENDIF ()

IF (CMAKE_COMPILER_IS_GNUCXX)
  # -Wno-maybe-uninitialized and -Wno-unused-but-set-parameter because gcc
  # implementations of these are buggy.
  SET (TFC_C_WARN_FLAGS "-Wformat=2 -Wall -Wextra -Wuninitialized -Wunreachable-code -Wno-unused-but-set-parameter -Wno-maybe-uninitialized -Wno-format-nonliteral")
  SET (TFC_CXX_WARN_FLAGS "${TFC_C_WARN_FLAGS} -Wstrict-null-sentinel -Woverloaded-virtual -Wnoexcept -Wnon-virtual-dtor -Winit-self -Wno-non-template-friend")
  SET (TFC_OPTIMIZATION_FLAGS "-O3 -ffast-math")

  SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TFC_C_WARN_FLAGS}")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TFC_CXX_WARN_FLAGS} -std=c++14")

  OPTION (ENABLE_STATIC_LIBGCC_LIBSTDCXX "Statically link libgcc and libstdc++" OFF)
  IF (ENABLE_STATIC_LIBGCC_LIBSTDCXX)
    SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static-libgcc")
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libgcc -static-libstdc++")
  ENDIF ()

  SET (CMAKE_C_FLAGS_DEBUG "-g -Og")
  SET (CMAKE_CXX_FLAGS_DEBUG "-g -Og")

  SET (CMAKE_C_FLAGS_RELWITHASSERTS "${TFC_OPTIMIZATION_FLAGS} -g -fno-omit-frame-pointer")
  SET (CMAKE_CXX_FLAGS_RELWITHASSERTS "${TFC_OPTIMIZATION_FLAGS} -g -fno-omit-frame-pointer")

  SET (CMAKE_C_FLAGS_RELWITHDEBINFO "${TFC_OPTIMIZATION_FLAGS} -DNDEBUG -g -fno-omit-frame-pointer")
  SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${TFC_OPTIMIZATION_FLAGS} -DNDEBUG -g -fno-omit-frame-pointer")

  SET (CMAKE_C_FLAGS_RELEASE "${TFC_OPTIMIZATION_FLAGS} -DNDEBUG -fno-omit-frame-pointer")
  SET (CMAKE_CXX_FLAGS_RELEASE "${TFC_OPTIMIZATION_FLAGS} -DNDEBUG -fno-omit-frame-pointer")

  IF (WIN32)
    SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mthreads")
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mthreads")

    IF (${CMAKE_TARGET_ARCHITECTURE} STREQUAL "i386")
      OPTION (ENABLE_3GB "Enable 3GB mode" ON)
      IF (ENABLE_3GB)
        SET (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--large-address-aware")
      ENDIF ()
    ENDIF ()
  ELSEIF (NOT APPLE)
    SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread")
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ENDIF ()

  OPTION (ENABLE_GLIBCXX_DEBUG "Enable _GLIBCXX_DEBUG" OFF)
  IF (ENABLE_GLIBCXX_DEBUG)
    SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_GLIBCXX_DEBUG")
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_DEBUG")
  ENDIF ()

  OPTION (ENABLE_DWARF2_DEBUG "Enable dwarf2 symbols" OFF)
  IF (ENABLE_DWARF2_DEBUG)
    SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -gdwarf-2")
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -gdwarf-2")
  ENDIF ()

  OPTION (ENABLE_PROFILING "Enable profiling" OFF)
  IF (ENABLE_PROFILING)
    SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pg")
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
    SET (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")
  ENDIF ()

ELSEIF (CMAKE_COMPILER_IS_CLANGXX)
  SET (TFC_C_WARN_FLAGS "-Wall -Wextra -Wuninitialized -Wno-parentheses-equality -Wno-deprecated-declarations")
  SET (TFC_CXX_WARN_FLAGS "${TFC_C_WARN_FLAGS} -Woverloaded-virtual -Wnon-virtual-dtor -Winit-self")
  SET (TFC_OPTIMIZATION_FLAGS "-O3 -ffast-math")

  SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TFC_C_WARN_FLAGS}")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TFC_CXX_WARN_FLAGS} -std=c++14")

  IF (NOT APPLE)
    SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread")
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ENDIF ()

  SET (CMAKE_C_FLAGS_DEBUG "-g")
  SET (CMAKE_CXX_FLAGS_DEBUG "-g")

  SET (CMAKE_C_FLAGS_RELWITHASSERTS "${TFC_OPTIMIZATION_FLAGS} -g -fno-omit-frame-pointer")
  SET (CMAKE_CXX_FLAGS_RELWITHASSERTS "${TFC_OPTIMIZATION_FLAGS} -g -fno-omit-frame-pointer")

  SET (CMAKE_C_FLAGS_RELWITHDEBINFO "${TFC_OPTIMIZATION_FLAGS} -DNDEBUG -g -fno-omit-frame-pointer")
  SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${TFC_OPTIMIZATION_FLAGS} -DNDEBUG -g -fno-omit-frame-pointer")

  SET (CMAKE_C_FLAGS_RELEASE "${TFC_OPTIMIZATION_FLAGS} -DNDEBUG -fno-omit-frame-pointer")
  SET (CMAKE_CXX_FLAGS_RELEASE "${TFC_OPTIMIZATION_FLAGS} -DNDEBUG -fno-omit-frame-pointer")

  IF (APPLE)
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")

    SET (CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14")
    SET (CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
  ENDIF()

ELSEIF (MSVC)
  # /MT     - Statically link with the visual c++ runtime
  # /EHsc   - Enable normal C++ exception handling
  # /bigobj - More sections in .obj files (Cannot build in Debug without it)
  # /wd4996 - Disable warnings about unsafe C functions
  # /wd4351 - Disable warnings about new behavior of default initialization of
  #           arrays (which is the correct behavior anyway)
  # /wd4800 - Disable warnings about using non-bool as true or false (useless
  #           performance warning)
  # /wd4244 - Disable warnings about type conversion loss of data, it's a nice
  #           warning, but it triggers on lots and lots of harmless things that no
  #           other compiler warns about, like passing an int as a float parameter
  # /wd4305 - Disable warnings about truncation from double to float
  # /wd4267 - Disable warnings about 64 - 32 bit truncation
  # /wd4456 - Disable warnings about hiding previous local declaration
  # /wd4503 - Silence warnings about MSVC generating a name so long it has to
  #           truncate it
  # /wd4250 - Silence "XX inherits YY via dominance"
  # /wd4624 - Silence implicitly deleted destructor warnings that show up when
  #           using unions in interesting ways.

  SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MT /EHsc /bigobj /wd4996 /wd4351 /wd4800 /wd4244 /wd4305 /wd4267 /wd4456 /wd4503 /wd4250 /wd4624")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MT /EHsc /bigobj /wd4996 /wd4351 /wd4800 /wd4244 /wd4305 /wd4267 /wd4456 /wd4503 /wd4250 /wd4624")

  SET (CMAKE_C_FLAGS_DEBUG "/Zi /Od")
  SET (CMAKE_CXX_FLAGS_DEBUG "/Zi /Od")

  SET (TFC_OPTIMIZATION_FLAGS "/Ox /fp:fast /fp:except- /GA /GS-")
  IF (${CMAKE_TARGET_ARCHITECTURE} STREQUAL "i386")
    SET (TFC_OPTIMIZATION_FLAGS "${TFC_OPTIMIZATION_FLAGS} /arch:SSE2")
  ENDIF ()

  SET (CMAKE_C_FLAGS_RELWITHASSERTS "${TFC_OPTIMIZATION_FLAGS} /Zi /Oy-")
  SET (CMAKE_CXX_FLAGS_RELWITHASSERTS "${TFC_OPTIMIZATION_FLAGS} /Zi /Oy-")

  SET (CMAKE_C_FLAGS_RELWITHDEBINFO "${TFC_OPTIMIZATION_FLAGS} /DNDEBUG /Zi /Oy-")
  SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${TFC_OPTIMIZATION_FLAGS} /DNDEBUG /Zi /Oy-")

  SET (CMAKE_C_FLAGS_RELEASE "${TFC_OPTIMIZATION_FLAGS} /DNDEBUG /Oy-")
  SET (CMAKE_CXX_FLAGS_RELEASE "${TFC_OPTIMIZATION_FLAGS} /DNDEBUG /Oy-")

ENDIF ()

IF (APPLE)
  ADD_DEFINITIONS(-DTFC_APPLE)

  SET (CMAKE_MODULE_LINKER_FLAGS "-flat_namespace -undefined suppress")
  SET (CMAKE_FRAMEWORK_PATH ${PROJECT_SOURCE_DIR}/../lib/osx)
  SET (CMAKE_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../lib/osx/include)
  SET (CMAKE_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/../lib/osx)
ELSEIF (LINUX)
  ADD_DEFINITIONS(-DTFC_LINUX)

  IF (${CMAKE_TARGET_ARCHITECTURE} STREQUAL "i386")
    SET (CMAKE_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../lib/linux32/include)
    SET (CMAKE_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/../lib/linux32)
  ENDIF ()

  IF (${CMAKE_TARGET_ARCHITECTURE} STREQUAL "x86_64")
    SET (CMAKE_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../lib/linux64/include)
    SET (CMAKE_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/../lib/linux64)
  ENDIF ()
ELSEIF (WIN32)
  ADD_DEFINITIONS(-DTFC_WINDOWS)

  IF (MSVC)
    ADD_DEFINITIONS(-DTFC_MSVC)
    IF (${CMAKE_TARGET_ARCHITECTURE} STREQUAL "i386")
      SET (CMAKE_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../lib/msvc32/include)
      SET (CMAKE_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/../lib/msvc32)
    ENDIF ()

    IF (${CMAKE_TARGET_ARCHITECTURE} STREQUAL "x86_64")
      SET (CMAKE_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../lib/msvc64/include)
      SET (CMAKE_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/../lib/msvc64)
    ENDIF ()
  ELSEIF (MINGW)
    ADD_DEFINITIONS(-DWINVER=0x0501)
    ADD_DEFINITIONS(-DTFC_MINGW)

    IF (${CMAKE_TARGET_ARCHITECTURE} STREQUAL "i386")
      SET (CMAKE_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../lib/mingw32/include)
      SET (CMAKE_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/../lib/mingw32)
    ENDIF ()

    IF (${CMAKE_TARGET_ARCHITECTURE} STREQUAL "x86_64")
      SET (CMAKE_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../lib/mingw64/include)
      SET (CMAKE_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/../lib/mingw64)
    ENDIF ()
  ENDIF ()

  SET (CMAKE_RC_COMPILER_INIT windres)
  ENABLE_LANGUAGE (RC)
  SET (CMAKE_RC_COMPILE_OBJECT
    "<CMAKE_RC_COMPILER> <FLAGS> -O coff <DEFINES> -i <SOURCE> -o <OBJECT>")
ENDIF ()

IF (APPLE)
  SET (BASE_LIBS)
ELSEIF (LINUX)
  SET (BASE_LIBS
      pthread
      dl
    )
ELSEIF (WIN32)
  SET (BASE_LIBS
      ws2_32
      winmm
      iphlpapi
      shlwapi
      dbghelp
    )
ENDIF ()

#INCLUDE_DIRECTORIES (SYSTEM)

SET (BUILD_SHARED_LIBS false)

# First set output dir for the generic no-config case (e.g. with linux)
SET (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../dist)

# Second, set output dir for multi-config builds (e.g. msvc)
FOREACH (OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
  STRING (TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
  SET (CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${PROJECT_SOURCE_DIR}/../dist)
ENDFOREACH (OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES)

SET (tfc-sim_HEADERS
  Config.hpp
  MifParse.hpp
)

SET (tfc-sim_SOURCES
  MifParse.cpp
  SComp.cpp
  sim.cpp
)

SET (tfc-parsemif_HEADERS
  Config.hpp
  MifParse.hpp
)

SET (tfc-parsemif_SOURCES
  MifParse.cpp
  parsemif.cpp
)

ADD_EXECUTABLE (tfc-sim ${tfc-sim_HEADERS} ${tfc-sim_SOURCES})

ADD_EXECUTABLE (tfc-parsemif ${tfc-parsemif_HEADERS} ${tfc-parsemif_SOURCES})
