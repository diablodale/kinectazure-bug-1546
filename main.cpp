// Copyright (c) Dale Phurrough
// Licensed under the MIT License

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <string_view>
#include <k4a/k4a.hpp>
#include <k4abt.hpp>
using namespace std::chrono;
using namespace std::string_view_literals;

int main(int argc, char *argv[]) {
    if (argc == 1) {
        std::cout << "Usage:    " << argv[0] << " gpu_index [gpu_type [wait_ms]]\n\n"
                  << "          gpu_index is integer\n"
                  << "          gpu_type is \"directml\" or \"cuda\" with default = directml\n"
                  << "          wait_ms is time in integer millisec between camera captures with default = 0\n\n"
                  << "Examples:\n"
                  << "          bug1546.exe 0                directml gpu 0, no wait\n"
                  << "          bug1546.exe 1                directml gpu 1, no wait\n"
                  << "          bug1546.exe 0 cuda           cuda gpu 0, no wait\n"
                  << "          bug1546.exe 0 directml 33    directml gpu 0, 33 millisecond wait (i.e. 30 fps)\n\n"
                  << "          Kinect is K4A_FRAMES_PER_SECOND_30, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_OFF" << std::endl;
        return 0;
    }

    try {
        k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        device_config.synchronized_images_only = false;

        k4a::device device = k4a::device::open(0);
        device.start_cameras(&device_config);
        k4a::calibration sensor_calibration = device.get_calibration(device_config.depth_mode, device_config.color_resolution);
        k4abt::tracker tracker = k4abt::tracker::create(sensor_calibration, {
            K4ABT_SENSOR_ORIENTATION_DEFAULT,
            (argc > 2) && ("cuda"sv == argv[2]) ? K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA : K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML,
            argc > 1 ? std::atoi(argv[1]) : 0,
            "dnn_model_2_0_op11.onnx"
        });

        const auto ping_wait = std::chrono::milliseconds(argc > 3 ? std::atoi(argv[3]) : 0);
        int bt_frame_count = 0;
        auto startTime = steady_clock::now();
        auto sleep_until = startTime;

        while (true) {
            std::this_thread::sleep_until(sleep_until);
            const auto now = steady_clock::now();
            const duration<float> sample_time = now - startTime;
            if (sample_time >= 2s) {
                const float bt_fps = bt_frame_count / sample_time.count();
                std::cout << "bodytrack fps " << bt_fps << std::endl;
                bt_frame_count = 0;
                startTime = steady_clock::now();
            }
            sleep_until = now + ping_wait;

            static const auto sensor_wait = std::chrono::milliseconds(0);
            static const auto bt_wait = std::chrono::milliseconds(5000);
            k4a::capture sensor_capture;
            if (device.get_capture(&sensor_capture, sensor_wait)) {
                if (!tracker.enqueue_capture(sensor_capture, bt_wait)) {
                    std::cout << "bt enqueue timeout" << std::endl;
                    continue;
                }

                k4abt::frame body_frame = tracker.pop_result(bt_wait);
                if (!body_frame) {
                    // likely bad state, since no bt result in 5 seconds, and
                    // if bt api eventually finishes, then bt output queue could grow deeper
                    std::cout << "bt pop timeout" << std::endl;
                    continue;
                }

                ++bt_frame_count;
                uint32_t num_bodies = body_frame.get_num_bodies();
                float dummy = static_cast<float>(num_bodies);
                for (uint32_t i = 0; i < num_bodies; ++i) {
                    k4abt_body_t body = body_frame.get_body(i);
                    dummy *= body.id * body.skeleton.joints[1].position.xyz.x;
                }
                (void)dummy;
            }
            else {
                // camera frame timeout
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Failed with exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
