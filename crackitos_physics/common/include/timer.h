#ifndef CRACKITOS_PHYSICS_COMMON_TIMER_H_
#define CRACKITOS_PHYSICS_COMMON_TIMER_H_

#include <chrono>

#include "commons.h"

namespace crackitos_physics::timer
{
    //TODO remove class, make static
    //TODO make reset function
    class Timer
    {
    public:
        Timer();

        void Tick() noexcept;

        [[nodiscard]] commons::fp DeltaTime() const noexcept;
        [[nodiscard]] commons::fp TotalTime() const noexcept;

        void SetFixedDeltaTime(commons::fp seconds) noexcept;
        bool FixedDeltaTimeStep() noexcept;
        [[nodiscard]] commons::fp FixedDeltaTime() const noexcept;

    private:
        std::chrono::high_resolution_clock::time_point start_time_;
        std::chrono::high_resolution_clock::time_point last_time_;
        std::chrono::duration<commons::fp> delta_time_{};
        std::chrono::duration<commons::fp> accumulated_time_{};

        std::chrono::duration<commons::fp> fixed_delta_time_{1.0f / 60.0f}; // Default 60 FPS
    };
} // namespace timer
#endif // CRACKITOS_PHYSICS_COMMON_TIMER_H_
