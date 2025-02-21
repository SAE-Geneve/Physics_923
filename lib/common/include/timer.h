#ifndef PHYSICS_923_LIB_COMMON_TIMER_H_
#define PHYSICS_923_LIB_COMMON_TIMER_H_

#include <chrono>

namespace physics923::timer
{
    class Timer
    {
    public:
        Timer();

        void Tick() noexcept;

        [[nodiscard]] float DeltaTime() const noexcept;
        [[nodiscard]] float TotalTime() const noexcept;

        void SetFixedDeltaTime(float seconds) noexcept;
        bool FixedDeltaTimeStep() noexcept;
        [[nodiscard]] float FixedDeltaTime() const noexcept;

    private:
        std::chrono::high_resolution_clock::time_point start_time_;
        std::chrono::high_resolution_clock::time_point last_time_;
        std::chrono::duration<float> delta_time_{};
        std::chrono::duration<float> accumulated_time_{};

        std::chrono::duration<float> fixed_delta_time_{1.0f / 60.0f}; // Default 60 FPS
    };
}
#endif // PHYSICS_923_LIB_COMMON_TIMER_H_
