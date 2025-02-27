#ifndef PHYSICS_SAMPLES_PLANET_SYSTEM_H_
#define PHYSICS_SAMPLES_PLANET_SYSTEM_H_

#include <vector>

#include "body.h"
#include "game_object.h"
#include "vec2.h"

namespace crackitos_physics::samples
{
    class PlanetSystem
    {
    private:
        static constexpr crackitos_physics::commons::fp kGravitationConstant_ = 0.0667f;
        static constexpr std::size_t kStartingPlanetsCount_ = 20;

        crackitos_physics::commons::fp star_mass_ = 10.f;
        crackitos_physics::commons::fp planet_mass_ = 3.f;

        physics::Body star_;
        std::vector<GameObject> planets_{};

        bool is_spawner_active_ = false;

    public:
        PlanetSystem() = default;
        ~PlanetSystem() = default;

        void Initialize();
        void Update(crackitos_physics::commons::fp delta_time, SDL_Color colour);
        void Clear();

        void CreatePlanet(math::Vec2f position, crackitos_physics::commons::fp radius, SDL_Color color);
        void UpdatePlanets(crackitos_physics::commons::fp delta_time);
        void UpdatePlanetsSIMD(crackitos_physics::commons::fp delta_time);
        void SpawnPlanets(SDL_Color colour);

        std::vector<GameObject> planets() { return planets_; }
        physics::Body* star() { return &star_; }

        void ToggleSpawner() { is_spawner_active_ = !is_spawner_active_; };
    };
} // namespace samples
#endif // PHYSICS_SAMPLES_PLANET_SYSTEM_H_
