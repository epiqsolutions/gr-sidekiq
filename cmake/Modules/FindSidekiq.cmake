# - Locate the Epiq Solutions Sidekiq SDK
# Exports on success:
#   Sidekiq_FOUND
#   Sidekiq_INCLUDE_DIRS
#   Sidekiq_LIBRARIES
#   Sidekiq_LIB_DIRS        # SDK support dir (use for RPATH)
#   OTHER_LIBS              # extra libs (gpiod on Z4; iio on some ARM suffixes)
#   SUFFIX                  # resolved SDK suffix
#
# Inputs (optional):
#   Sidekiq_ROOT or env Sidekiq_DIR (defaults to $HOME/sidekiq_sdk_current)
#   SUFFIX (override detection: z4, z3u, aarch64, x86_64.gcc, ...)

if (NOT Sidekiq_FOUND)

  # ---- SDK root ----
  if (NOT DEFINED Sidekiq_ROOT OR "${Sidekiq_ROOT}" STREQUAL "")
    if (DEFINED ENV{Sidekiq_DIR} AND NOT "$ENV{Sidekiq_DIR}" STREQUAL "")
      set(Sidekiq_ROOT "$ENV{Sidekiq_DIR}")
    else()
      set(Sidekiq_ROOT "$ENV{HOME}/sidekiq_sdk_current")
    endif()
  endif()
  file(TO_CMAKE_PATH "${Sidekiq_ROOT}" Sidekiq_ROOT)
  set(ENV{Sidekiq_DIR} "${Sidekiq_ROOT}")

  # ---- Headers ----
  find_path(Sidekiq_INCLUDE_DIR
    NAMES sidekiq_api.h
    HINTS "${Sidekiq_ROOT}/include" "${Sidekiq_ROOT}/sidekiq_core/inc"
    PATHS /usr/local/include /usr/include /opt/include /opt/local/include)
  if (NOT Sidekiq_INCLUDE_DIR)
    message(FATAL_ERROR "Sidekiq headers not found (checked ${Sidekiq_ROOT}/include and sidekiq_core/inc)")
  endif()
  set(Sidekiq_INCLUDE_DIRS "${Sidekiq_INCLUDE_DIR}")

  # ---- Arch / suffix detection ----
  execute_process(COMMAND uname -m OUTPUT_VARIABLE cpu_arch OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(STRIP "${cpu_arch}" cpu_arch)
  message(STATUS "cpu_arch is: '${cpu_arch}'")

  if (NOT DEFINED SUFFIX OR "${SUFFIX}" STREQUAL "")
    if (EXISTS "${Sidekiq_ROOT}/lib/support/z4")
      set(SUFFIX "z4")
    else()
      set(SUFFIX "none")
      file(GLOB _SK_ARCH "${Sidekiq_ROOT}/lib/libsidekiq__*.a")
      foreach(_arc IN LISTS _SK_ARCH)
        get_filename_component(_nm "${_arc}" NAME_WE)  # libsidekiq__<suffix>
        string(REPLACE "libsidekiq__" "" _sfx "${_nm}")
        if (EXISTS "${Sidekiq_ROOT}/lib/support/${_sfx}")
          set(SUFFIX "${_sfx}")
          break()
        endif()
      endforeach()
    endif()
  endif()
  if ("${SUFFIX}" STREQUAL "none")
    message(FATAL_ERROR "Could not determine SDK suffix. Pass -DSUFFIX=<suffix> (e.g., z4, z3u, aarch64, x86_64.gcc).")
  endif()

  set(Sidekiq_LIB_DIRS "${Sidekiq_ROOT}/lib/support/${SUFFIX}/usr/lib/epiq")
  set(_PKGCONFIG_DIR   "${Sidekiq_LIB_DIRS}/pkgconfig")

  # ---- Map suffix to strategy ----
  set(otherlib "none")
  set(_use_shared FALSE)
  set(libname "")

  if ("${cpu_arch}" STREQUAL "x86_64")
    set(libname "libsidekiq__x86_64.gcc.a")
  elseif ("${SUFFIX}" STREQUAL "msiq-x40")
    set(libname "libsidekiq__msiq-x40.a")
  elseif ("${SUFFIX}" STREQUAL "msiq-g20g40")
    set(libname "libsidekiq__msiq-g20g40.a")
  elseif ("${SUFFIX}" STREQUAL "z3u")
    set(libname "libsidekiq__z3u.a")
    set(otherlib "iio")
  elseif ("${SUFFIX}" STREQUAL "aarch64")
    set(libname "libsidekiq__aarch64.a")
    set(otherlib "iio")
  elseif ("${SUFFIX}" STREQUAL "aarch64.gcc6.3")
    set(libname "libsidekiq__aarch64.gcc6.3.a")
    set(otherlib "iio")
  elseif ("${SUFFIX}" STREQUAL "arm_cortex-a9.gcc7.2.1_gnueabihf")
    set(libname "libsidekiq__arm_cortex-a9.gcc7.2.1_gnueabihf.a")
    set(otherlib "iio")
  elseif ("${SUFFIX}" STREQUAL "z4")
    set(_use_shared TRUE) # shared sidekiq + bundled gpiod from SDK support dir
  else()
    message(FATAL_ERROR "Invalid platform suffix '${SUFFIX}'")
  endif()

  message(STATUS "Detected SDK SUFFIX: ${SUFFIX}")

  # ---- Locate libraries ----
  unset(Sidekiq_LIBRARY CACHE)
  unset(OTHER_LIBS CACHE)
  set(PKGCONFIG_LIBS "")

  if (_use_shared)
    find_library(Sidekiq_LIBRARY
      NAMES sidekiq-dev-z4-1 sidekiq
      HINTS "${Sidekiq_LIB_DIRS}" "${Sidekiq_ROOT}/lib"
      NO_DEFAULT_PATH)
    if (NOT Sidekiq_LIBRARY)
      message(FATAL_ERROR "Z4: shared sidekiq .so not found in ${Sidekiq_LIB_DIRS}")
    endif()

    find_library(GPIOD_SDK_LIBRARY NAMES gpiod
      HINTS "${Sidekiq_LIB_DIRS}" NO_DEFAULT_PATH)
    if (NOT GPIOD_SDK_LIBRARY)
      message(FATAL_ERROR "Z4: libgpiod not found in ${Sidekiq_LIB_DIRS}")
    endif()
    set(OTHER_LIBS "${GPIOD_SDK_LIBRARY}")

    if (EXISTS "${_PKGCONFIG_DIR}")
      set(ENV{PKG_CONFIG_PATH} "${_PKGCONFIG_DIR}:$ENV{PKG_CONFIG_PATH}")
      message(STATUS "Z4: PKG_CONFIG_PATH = $ENV{PKG_CONFIG_PATH}")
    endif()
  else()
    find_library(Sidekiq_LIBRARY
      NAMES ${libname}
      HINTS "${Sidekiq_ROOT}/lib"
      PATHS /usr/local/lib /usr/lib /usr/lib64)
    if (NOT Sidekiq_LIBRARY)
      message(FATAL_ERROR "Sidekiq static library not found (expected ${libname})")
    endif()

    if (NOT "${otherlib}" STREQUAL "none")
      find_library(OTHER_LIBS
        NAMES ${otherlib}
        HINTS "${Sidekiq_LIB_DIRS}"
        PATHS /usr/lib/epiq /usr/local/lib /usr/lib /opt/lib /opt/local/lib)
      if (NOT OTHER_LIBS)
        message(FATAL_ERROR "Required extra library '${otherlib}' not found for suffix ${SUFFIX}")
      endif()
    endif()
  endif()

  set(Sidekiq_LIBRARIES "${Sidekiq_LIBRARY}")

  include(FindPackageHandleStandardArgs)
  if (_use_shared)
    find_package_handle_standard_args(Sidekiq DEFAULT_MSG
      Sidekiq_LIBRARY Sidekiq_INCLUDE_DIR)
  else()
    if (NOT "${otherlib}" STREQUAL "none")
      find_package_handle_standard_args(Sidekiq DEFAULT_MSG
        Sidekiq_LIBRARY Sidekiq_INCLUDE_DIR OTHER_LIBS)
    else()
      find_package_handle_standard_args(Sidekiq DEFAULT_MSG
        Sidekiq_LIBRARY Sidekiq_INCLUDE_DIR)
    endif()
  endif()

  mark_as_advanced(Sidekiq_INCLUDE_DIRS Sidekiq_LIBRARIES OTHER_LIBS PKGCONFIG_LIBS)
endif()
