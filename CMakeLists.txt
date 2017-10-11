
cmake_minimum_required(VERSION 3.0)
project(msaecwebrtc VERSION 1.0.0 LANGUAGES C CXX)
set(PACKAGE_TARNAME "msaecwebrtc")

####################################
###### BC toolbox integration ######
####################################
set(PACKAGE "${PROJECT_NAME}")
set(PACKAGE_NAME "${PROJECT_NAME}")
set(PACKAGE_VERSION "${PROJECT_VERSION}")
set(PACKAGE_STRING "${PACKAGE_NAME} ${PACKAGE_VERSION}")
set(PACKAGE_BUGREPORT "jchavanton@aec.com")
set(PACKAGE_URL "")
set(VERSION "${PACKAGE_VERSION}")

option(ENABLE_SHARED "Build shared library." YES)
option(ENABLE_STATIC "Build static library." YES)

include(GNUInstallDirs)

if(NOT CMAKE_INSTALL_RPATH AND CMAKE_INSTALL_PREFIX)
	set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_FULL_LIBDIR})
	message(STATUS "Setting install rpath to ${CMAKE_INSTALL_RPATH}")
endif()

if(LINPHONE_BUILDER_GROUP_EXTERNAL_SOURCE_PATH_BUILDERS)
	include("${EP_ms2_CONFIG_DIR}/Mediastreamer2Config.cmake")
else()
	find_package(Mediastreamer2 REQUIRED)
endif()

add_definitions(-DHAVE_CONFIG_H)
if(WIN32)
	add_definitions(-DWEBRTC_WIN)
endif()
if(ANDROID)
	add_definitions("-DANDROID")
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "WindowsPhone" AND CMAKE_GENERATOR_PLATFORM STREQUAL "ARM")
	add_definitions(
		-D__ARMEL__
		-DWEBRTC_ARCH_ARM
	)
elseif(CMAKE_SYSTEM_NAME STREQUAL "WindowsStore" AND CMAKE_GENERATOR MATCHES "^.* ARM$")
	add_definitions(
		-D__ARMEL__
		-DWEBRTC_ARCH_ARM
	)
endif()
if(UNIX)
	add_definitions(-DWEBRTC_POSIX)
	if(APPLE)
		add_definitions(-DWEBRTC_MAC -DWEBRTC_THREAD_RR -DWEBRTC_CLOCK_TYPE_REALTIME)
	else()
		add_definitions(-DWEBRTC_LINUX -DWEBRTC_THREAD_RR)
	endif()
endif()

string(REGEX MATCH "^(arm*|aarch64)" FIXED_POINT_PROCESSOR "${CMAKE_SYSTEM_PROCESSOR}")
if(FIXED_POINT_PROCESSOR)
	add_definitions(-D__ARMEL__)
	set(ARM true)
endif()
####################################
set(CMAKE_VERBOSE_MAKEFILE ON)

###### source files ########

set(ENABLE_AECM true)

if(ANDROID AND CMAKE_SYSTEM_PROCESSOR STREQUAL "armeabi-v7a")
	option(ENABLE_NEON "Active NEON detection and include neon source files." YES)
endif()

set(WEBRTC_ROOT_DIR ".")
set(WEBRTC_SRC_DIR "webrtc")
include_directories(${WEBRTC_ROOT_DIR})
set(COMMON_AUDIO_SRC_DIR "${WEBRTC_SRC_DIR}/common_audio")
include_directories(${COMMON_AUDIO_SRC_DIR})

set(SIGNAL_PROCESSING_SRC_DIR "${COMMON_AUDIO_SRC_DIR}/signal_processing")

set(SYSTEM_WRAPPERS_SRC_DIR "${WEBRTC_SRC_DIR}/system_wrappers/source")
include_directories("${SYSTEM_WRAPPERS_SRC_DIR}/../include")
set(UTILITY_SRC_DIR "${WEBRTC_SRC_DIR}/modules/audio_processing/utility")
set(UTILITY_SRC_DIR "${WEBRTC_SRC_DIR}/modules/audio_processing/utility")
set(AECM_SRC_DIR "${WEBRTC_SRC_DIR}/modules/audio_processing/aecm")

enable_language(ASM)
if(CMAKE_C_COMPILER_TARGET)
	set(CMAKE_ASM_TARGET_FLAG " --target=${CMAKE_C_COMPILER_TARGET}")
endif()

set(AEC_WEBRTC_SRCS msaecwebrtc.c)

if(ENABLE_AECM)
	set(AECM_CC_SRCS
		${AECM_SRC_DIR}/echo_control_mobile.cc
		${AECM_SRC_DIR}/aecm_core.cc
		${AECM_SRC_DIR}/aecm_core_c.cc
		${UTILITY_SRC_DIR}/delay_estimator.cc
		${UTILITY_SRC_DIR}/delay_estimator_wrapper.cc
	)
	set_source_files_properties(${AECM_CC_SRCS} PROPERTIES COMPILE_FLAGS "--std=c++11 ")
	set(AECM_SRCS
		aec.c
		${AECM_SRC_DIR}/echo_control_mobile.cc
		${AECM_SRC_DIR}/aecm_core.cc
		${AECM_SRC_DIR}/aecm_core_c.cc
		${UTILITY_SRC_DIR}/delay_estimator.cc
		${UTILITY_SRC_DIR}/delay_estimator_wrapper.cc
		${COMMON_AUDIO_SRC_DIR}/ring_buffer.c
		${SIGNAL_PROCESSING_SRC_DIR}/division_operations.c
		${SIGNAL_PROCESSING_SRC_DIR}/real_fft.c
		${SIGNAL_PROCESSING_SRC_DIR}/complex_fft.c
		${SIGNAL_PROCESSING_SRC_DIR}/spl_init.c
		${SIGNAL_PROCESSING_SRC_DIR}/randomization_functions.c
		${SIGNAL_PROCESSING_SRC_DIR}/min_max_operations.c
		${SIGNAL_PROCESSING_SRC_DIR}/cross_correlation.c
		${SIGNAL_PROCESSING_SRC_DIR}/downsample_fast.c
		${SIGNAL_PROCESSING_SRC_DIR}/vector_scaling_operations.c
	)

	if(ENABLE_NEON)
		message("building NEON sources")
		set(NEON_SRCS
			${SYSTEM_WRAPPERS_SRC_DIR}/cpu_features_android.c
			${AECM_SRC_DIR}/aecm_core_neon.cc
			${SIGNAL_PROCESSING_SRC_DIR}/min_max_operations_neon.c
			${SIGNAL_PROCESSING_SRC_DIR}/cross_correlation_neon.c
			${SIGNAL_PROCESSING_SRC_DIR}/downsample_fast_neon.c
		)
		set_source_files_properties(${NEON_SRCS} PROPERTIES COMPILE_FLAGS "-mfpu=neon -mfloat-abi=softfp ")
		set(AECM_SRCS ${AECM_SRCS} ${NEON_SRCS})
		add_definitions(-DWEBRTC_HAS_NEON) # Activation of NEON now for all ARMv7 https://bugs.chromium.org/p/chromium/issues/detail?id=448055
	endif()

	# INT32_MAX and INT32_MIN
	add_definitions(-D __STDC_LIMIT_MACROS -D __STDC_FORMAT_MACROS)

	if(ARM)
		set(AECM_ASM_SRCS
			${SIGNAL_PROCESSING_SRC_DIR}/complex_bit_reverse_arm.S
			${SIGNAL_PROCESSING_SRC_DIR}/spl_sqrt_floor_arm.S
		)
		set_source_files_properties(${AECM_ASM_SRCS} PROPERTIES COMPILE_FLAGS ${CMAKE_ASM_TARGET_FLAG})
	else()
		list(APPEND AECM_SRCS
			${SIGNAL_PROCESSING_SRC_DIR}/complex_bit_reverse.c
			${SIGNAL_PROCESSING_SRC_DIR}/spl_sqrt_floor.c
		)
	endif()
	include_directories(${AECM_SRC_DIR})
endif()

set(SOURCE_FILES ${AEC_WEBRTC_SRCS} ${AECM_CC_SRCS} ${AECM_SRCS} ${AECM_ASM_SRCS})



####################################
###### BC toolbox integration ######
####################################
#add_library(msaecwebrtc MODULE ${SOURCE_FILES})
set(LIBS ${MEDIASTREAMER2_LIBRARIES})
if(ENABLE_STATIC)
	add_library(msaecwebrtc-static STATIC ${SOURCE_FILES})
	set_target_properties(msaecwebrtc-static PROPERTIES OUTPUT_NAME msaecwebrtc)
	target_link_libraries(msaecwebrtc-static ${LIBS})
	install(TARGETS msaecwebrtc-static
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/mediastreamer/plugins
		PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
	)
endif()
if(ENABLE_SHARED)
	add_library(msaecwebrtc MODULE ${SOURCE_FILES})
	target_link_libraries(msaecwebrtc ${LIBS})
	if(MSVC)
		if(CMAKE_BUILD_TYPE STREQUAL "Debug")
			install(FILES ${CMAKE_CURRENT_BINARY_DIR}/Debug/libmsaecwebrtc.pdb
				DESTINATION ${CMAKE_INSTALL_BINDIR}
				PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
			)
		endif()
		set_target_properties(msaecwebrtc PROPERTIES PREFIX "lib")
	endif()
	install(TARGETS msaecwebrtc
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/mediastreamer/plugins
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/mediastreamer/plugins
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/mediastreamer/plugins
		PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
	)
endif()
#####################################
