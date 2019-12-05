#include "main.h"


// ============================================================================
class frame_queue
{
public:
    struct cancelled {};

public:
    frame_queue();

    void push(cv::Mat const& image);
    cv::Mat pop();

    void cancel();

private:
    std::queue<cv::Mat> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    bool cancelled_;
};
// ----------------------------------------------------------------------------
frame_queue::frame_queue()
    : cancelled_(false)
{
}
// ----------------------------------------------------------------------------
void frame_queue::cancel()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    cancelled_ = true;
    cond_.notify_all();
}
// ----------------------------------------------------------------------------
void frame_queue::push(cv::Mat const& image)
{
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push(image);
    cond_.notify_one();
}
// ----------------------------------------------------------------------------
cv::Mat frame_queue::pop()
{
    std::unique_lock<std::mutex> mlock(mutex_);

    while (queue_.empty()) {
        if (cancelled_) {
            throw cancelled();
        }
        cond_.wait(mlock);
        if (cancelled_) {
            throw cancelled();
        }
    }

    cv::Mat image(queue_.front());
    queue_.pop();
    return image;
}
// ============================================================================
