find_path(
    KINECT3_INCLUDES
    NAMES k4a/k4a.h k4a/k4a.hpp
    PATHS ENV KINECTSDK30_DIR
    PATH_SUFFIXES include
)
find_path(
    KINECT3BODY_INCLUDES
    NAMES k4abt.h k4abt.hpp
    PATHS ENV KINECTSDK30BODY_DIR
    PATH_SUFFIXES include
)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    find_path(
        KINECT3_LIBRARIES
        NAMES k4a.lib
        PATHS ENV KINECTSDK30_DIR
        PATH_SUFFIXES windows-desktop/amd64/release/lib
    )
    find_path(
        KINECT3_RUNTIME
        NAMES k4a.dll #depthengine_2_0.dll
        PATHS ENV KINECTSDK30_DIR
        PATH_SUFFIXES windows-desktop/amd64/release/bin
    )
    find_path(
        KINECT3BODY_LIBRARIES
        NAMES k4abt.lib
        PATHS ENV KINECTSDK30BODY_DIR
        PATH_SUFFIXES windows-desktop/amd64/release/lib
    )
    find_path(
        KINECT3BODY_RUNTIME
        NAMES k4abt.dll #onnxruntime.dll dnn_model_2_0.onnx
        PATHS ENV KINECTSDK30BODY_DIR
        PATH_SUFFIXES windows-desktop/amd64/release/bin
    )
endif()

file(TO_CMAKE_PATH "$ENV{KINECTSDK30BODY_DIR}" KINECTSDK30BODY_DIR)
