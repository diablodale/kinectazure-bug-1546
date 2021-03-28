# (c) Dale Phurrough
# Licensed under the MIT License

cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW) # enable Msft VC runtime library selection
cmake_policy(SET CMP0069 NEW) # more INTERPROCEDURAL_OPTIMIZATION support

# cmake support files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# project overall
project(bug1546
    DESCRIPTION "Kinect v3 (Azure) BT bug 1546"
    HOMEPAGE_URL https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1546
    LANGUAGES CXX
)

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Bug repro only 64-bit compile")
endif()
#if (NOT MSVC_VERSION EQUAL 1928)
#    message(FATAL_ERROR "Bug repo only Visual Studio 16.9.*; you are not using that version")
#endif()
if (NOT CMAKE_CXX_COMPILER_VERSION STREQUAL "19.28.29913.0")
    message(WARNING "Bug repro isolated on Visual Studio 16.9.2; you are not using that version")
endif()

# Packages
include("cmake/find-kinect.cmake")

# language settings
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
# fix limitations of VSCode intellisense
add_compile_definitions(
    $<$<CONFIG:Debug>:_DEBUG>
    _MT
    _DLL
)

string(REGEX REPLACE " /W[0-4]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(MSVC_ALL_WARNINGS
    "/W4"
    "/wd4200" # allow zero length arrays
    "/wd4324" # structure was padded due to alignment specifier
    # /wd4814 # warning about constexpr not being const in c++14
)
set(MSVC_WARNINGS_AS_ERRORS "/WX")
set(MSVC_HASH_SRC_IN_PDB "/ZH:SHA_256")
set(MSVC_UTF8 "/utf-8")
set(DISABLE_PRECOMPILE_HEADERS "/Y-")
set(MSVC_CPU_LEVEL "/arch:AVX")
set(MSVC_LOOP_PARALLEL_REPORT "/Qpar-report:1" "/Qvec-report:1")
set(MSVC_CTRL_FLOW_GUARD "/guard:cf")
set(MSVC_SECURITY_CHECKS_BROAD "/sdl")
set(MSVC_SECURITY_CHECKS_NO_BUFFER "/GS-")
set(MSVC_COMDAT_OPTIMIZE "/Gy" "/Gw")
set(MSVC_DEBUG_OPT_CODE "/Zo")
set(MSVC_INTEL_JCC_ERRATUM "/QIntel-jcc-erratum")
add_compile_options(
    ${MSVC_ALL_WARNINGS}
    ${MSVC_WARNINGS_AS_ERRORS}
    ${MSVC_HASH_SRC_IN_PDB}
    ${MSVC_UTF8}
    ${DISABLE_PRECOMPILE_HEADERS}
    ${MSVC_CPU_LEVEL}
    ${MSVC_LOOP_PARALLEL_REPORT}
    ${MSVC_CTRL_FLOW_GUARD}
    "$<IF:$<CONFIG:Debug>,${MSVC_SECURITY_CHECKS_BROAD},${MSVC_SECURITY_CHECKS_NO_BUFFER}>"
    "$<IF:$<CONFIG:Debug>,,${MSVC_COMDAT_OPTIMIZE}>"
    "$<$<CONFIG:RelWithDebInfo>:${MSVC_DEBUG_OPT_CODE}>"
    ${MSVC_INTEL_JCC_ERRATUM}
)

# main target
add_executable(${PROJECT_NAME}
    "main.cpp"
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE WINVER=0x0A00 _WIN32_WINNT=0x0A00 NTDDI_VERSION=0x0A000005 # WinKitSDK min supported version
    PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX _USE_MATH_DEFINES # WinSDK tweaks
    PRIVATE UNICODE _UNICODE #enables Windows and Cruntime headers to use widechar defaults for text macros/functions
    PRIVATE _SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION True # linker option /LTCG
    INTERPROCEDURAL_OPTIMIZATION_DEBUG False
)
target_include_directories(${PROJECT_NAME}
    BEFORE
	PRIVATE ${KINECT3_INCLUDES} ${KINECT3BODY_INCLUDES}
)
target_link_options(${PROJECT_NAME}
    PRIVATE /NODEFAULTLIB:VCOMP$<$<CONFIG:Debug>:D>.LIB # ensure no dependency on OpenMP runtime DLLs
    PRIVATE /DELAYLOAD:k4a.dll /DELAYLOAD:k4abt.dll
)
target_link_libraries(${PROJECT_NAME}
    PRIVATE delayimp.lib
    PRIVATE "${KINECT3_LIBRARIES}/k4a.lib"
    PRIVATE "${KINECT3BODY_LIBRARIES}/k4abt.lib"
)

# copy Kinect runtimes into build directory
file(
    GLOB KINECT_DEPTH_ENGINE
    LIST_DIRECTORIES false
    CONFIGURE_DEPENDS
    "${KINECT3_RUNTIME}/depthengine*.dll"
)
file(
    COPY
        "${KINECT3_RUNTIME}/k4a.dll"
        "${KINECT_DEPTH_ENGINE}"
        "${KINECT3BODY_RUNTIME}/directml.dll"
        "${KINECT3BODY_RUNTIME}/dnn_model_2_0_op11.onnx"
        "${KINECT3BODY_RUNTIME}/dnn_model_2_0_lite_op11.onnx"
        "${KINECT3BODY_RUNTIME}/k4abt.dll"
        "${KINECT3BODY_RUNTIME}/onnxruntime.dll"
        "${KINECT3BODY_RUNTIME}/onnxruntime_providers_shared.dll"
        "${KINECTSDK30BODY_DIR}/../tools/cudart64_110.dll"
        "${KINECTSDK30BODY_DIR}/../tools/cufft64_10.dll"
        "${KINECTSDK30BODY_DIR}/../tools/cublas64_11.dll"
        "${KINECTSDK30BODY_DIR}/../tools/cublasLt64_11.dll"
        "${KINECTSDK30BODY_DIR}/../tools/cudnn64_8.dll"
        # below for cuda and tensorrt
        "${KINECTSDK30BODY_DIR}/../tools/cudnn_ops_infer64_8.dll"
        "${KINECTSDK30BODY_DIR}/../tools/cudnn_cnn_infer64_8.dll"
        # below for tensorrt
        "${KINECTSDK30BODY_DIR}/../tools/onnxruntime_providers_tensorrt.dll"
        "${KINECTSDK30BODY_DIR}/../tools/nvrtc64_111_0.dll"
        "${KINECTSDK30BODY_DIR}/../tools/nvrtc-builtins64_111.dll"
        "${KINECTSDK30BODY_DIR}/../tools/nvinfer.dll"
        "${KINECTSDK30BODY_DIR}/../tools/nvinfer_plugin.dll"
        "${KINECTSDK30BODY_DIR}/../tools/myelin64_1.dll"
    DESTINATION "${PROJECT_BINARY_DIR}"
    NO_SOURCE_PERMISSIONS
)
