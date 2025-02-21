#include "timer.h"

namespace physics923::timer
{
    Timer::Timer()
    {
        start_time_ = std::chrono::high_resolution_clock::now();
        last_time_ = start_time_;
    }

    void Timer::Tick() noexcept
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        delta_time_ = current_time - last_time_;
        accumulated_time_ += delta_time_;
        last_time_ = current_time;
    }

    float Timer::DeltaTime() const noexcept
    {
        return delta_time_.count();
    }

    float Timer::TotalTime() const noexcept
    {
        return std::chrono::duration_cast<std::chrono::duration<float>>(
            std::chrono::high_resolution_clock::now() - start_time_).count();
    }

    void Timer::SetFixedDeltaTime(float seconds) noexcept
    {
        fixed_delta_time_ = std::chrono::duration<float>(seconds);
    }

    bool Timer::FixedDeltaTimeStep() noexcept
    {
        if (accumulated_time_ >= fixed_delta_time_)
        {
            accumulated_time_ -= fixed_delta_time_;
            return true;
        }
        return false;
    }

    float Timer::FixedDeltaTime() const noexcept
    {
        return fixed_delta_time_.count();
    }
}