#ifndef PHYSICS_923_LIB_COMMON_TIMER_H_
#define PHYSICS_923_LIB_COMMON_TIMER_H_

#include <chrono>
#include "commons.h"

namespace physics923::timer
{
    class Timer
    {
    public:
        Timer();

        void Tick() noexcept;

        [[nodiscard]] physics923::commons::fp DeltaTime() const noexcept;
        [[nodiscard]] physics923::commons::fp TotalTime() const noexcept;

        void SetFixedDeltaTime(physics923::commons::fp seconds) noexcept;
        bool FixedDeltaTimeStep() noexcept;
        [[nodiscard]] physics923::commons::fp FixedDeltaTime() const noexcept;

    private:
        std::chrono::high_resolution_clock::time_point start_time_;
        std::chrono::high_resolution_clock::time_point last_time_;
        std::chrono::duration<physics923::commons::fp> delta_time_{};
        std::chrono::duration<physics923::commons::fp> accumulated_time_{};

        std::chrono::duration<physics923::commons::fp> fixed_delta_time_{1.0f / 60.0f}; // Default 60 FPS
    };
}
#endif // PHYSICS_923_LIB_COMMON_TIMER_H_
