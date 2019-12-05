// ============================================================================
#include "main.h"

class storage_worker
{
public:
    storage_worker(frame_queue& queue
        , int32_t id
        , std::string const& file_name
        , int32_t fourcc
        , double fps
        , cv::Size frame_size
        , bool is_color = true);

    void run();

    double total_time_ms() const { return total_time_ / 1000.0; }

private:
    frame_queue& queue_;

    int32_t id_;

    std::string file_name_;
    int32_t fourcc_;
    double fps_;
    cv::Size frame_size_;
    bool is_color_;

    double total_time_;
};
// ----------------------------------------------------------------------------
storage_worker::storage_worker(frame_queue& queue
    , int32_t id
    , std::string const& file_name
    , int32_t fourcc
    , double fps
    , cv::Size frame_size
    , bool is_color)
    : queue_(queue)
    , id_(id)
    , file_name_(file_name)
    , fourcc_(fourcc)
    , fps_(fps)
    , frame_size_(frame_size)
    , is_color_(is_color)
    , total_time_(0.0)
{
}
// ----------------------------------------------------------------------------
void storage_worker::run()
{
    cv::VideoWriter writer(file_name_, 0, fps_, frame_size_, is_color_);
    try {
        int32_t frame_count(0);
        for (;;) {
            cv::Mat image(queue_.pop());
            if (!image.empty()) {
                high_resolution_clock::time_point t1(high_resolution_clock::now());

                ++frame_count;
                writer.write(image);
             //   usleep (200);
                high_resolution_clock::time_point t2(high_resolution_clock::now());
                double dt_us(static_cast<double>(duration_cast<microseconds>(t2 - t1).count()));
                total_time_ += dt_us;

                std::cout << "Worker " << id_ << " stored image #" << frame_count
                    << " in " << (dt_us / 1000.0) << " ms" << std::endl;
            }
        }
    } catch (frame_queue::cancelled& /*e*/) {
        // Nothing more to process, we're done
        std::cout << "Queue " << id_ << " cancelled, worker finished." << std::endl;
    }
}
// ============================================================================
