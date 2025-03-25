#include "planet_system.h"

#include <SDL_mouse.h>

#include "display.h"
#include "four_vec2.h"
#include "random.h"
#include "distance.h"

namespace crackitos_physics::samples {
void PlanetSystem::Initialize() {
  Clear();
  star_ = physics::Body(physics::BodyType::Static,
                        math::Vec2f(kWindowWidth / 2.f, kWindowHeight / 2.f),
                        math::Vec2f::Zero(),
                        false,
                        star_mass_
  );

  for (std::size_t i = 0; i < kStartingPlanetsCount_; i++) {
    constexpr crackitos_physics::commons::fp margin = 20.0f;
    const math::Vec2f position(random::Range(margin, kWindowWidth - margin),
                               random::Range(margin, kWindowHeight - margin));
    const crackitos_physics::commons::fp radius = random::Range(5.f, 20.f);
    const uint8_t alpha = random::Range(10, 255);

    CreatePlanet(position, radius, SDL_Color{255, 13, 132, alpha});
  }

  is_spawner_active_ = false;
}

void PlanetSystem::Update(crackitos_physics::commons::fp delta_time, SDL_Color colour) {
  if (is_spawner_active_) {
    SpawnPlanets(colour);
  }
  UpdatePlanetsSIMD(delta_time);
}

void PlanetSystem::Clear() {
  // Clear the vector of planets
  planets_.clear();

  // Reset the star
  star_ = physics::Body();

  // Deactivate the spawner
  is_spawner_active_ = false;
}

void PlanetSystem::CreatePlanet(const math::Vec2f position, const crackitos_physics::commons::fp radius,
                                const SDL_Color color) {
  math::Vec2f u = star_.position() - position;
  crackitos_physics::commons::fp r = u.Magnitude();

  if (r > 0) {
    // Calculate angular velocity magnitude based on gravitational force
    math::Vec2f tangential_direction = math::Vec2f(-u.y, u.x).Normalized();
    crackitos_physics::commons::fp orbital_velocity = std::sqrt(kGravitationConstant_ * (star_mass_ / r));

    // Calculate angular velocity
    math::Vec2f angular_velocity = tangential_direction * orbital_velocity;
    physics::Body body(physics::BodyType::Dynamic, position,
                       angular_velocity,
                       false,
                       planet_mass_
    );
    //Random mass: random::Range(1.0f, 50.0f)

    auto planet = GameObject(body, radius, color);
    planets_.push_back(planet);
  }
}

void PlanetSystem::UpdatePlanets(crackitos_physics::commons::fp delta_time) {
  for (auto &planet : planets_) {
    math::Vec2f u = star_.position() - planet.position();
    const crackitos_physics::commons::fp r = u.Magnitude();

    if (r > 0) {
      const crackitos_physics::commons::fp force_magnitude = kGravitationConstant_ * (planet_mass_ *
          star_mass_ / (r * r));
      const math::Vec2f force = force_magnitude * u.Normalized();
      planet.body().ApplyForce(force);
    }

    planet.body().Update(delta_time);
  }
}

void PlanetSystem::UpdatePlanetsSIMD(crackitos_physics::commons::fp delta_time) {
  const std::size_t simdSize = planets_.size() / 4 * 4;

  for (std::size_t i = 0; i < simdSize; i += 4) {
    //Load four planet positions and calculate vector u to the star for each
    math::FourVec2f planetPositions = {
        planets_[i].position(),
        planets_[i + 1].position(),
        planets_[i + 2].position(),
        planets_[i + 3].position()
    };

    math::FourVec2f u = math::FourVec2f({
                                            star_.position(), star_.position(), star_.position(), star_.position()
                                        }) - planetPositions;

    //Calculate distance to the star and force magnitudes
    std::array<crackitos_physics::commons::fp, 4> distances = u.Magnitude();
    math::FourVec2f normalizedU = u.Normalize();

    std::array<crackitos_physics::commons::fp, 4> forceMagnitudes = {};
    for (int j = 0; j < 4; ++j) {
      if (distances[j] > 0) {
        forceMagnitudes[j] = kGravitationConstant_ * (planets_[i + j].body().mass() * star_mass_ / (
            distances[j] * distances[j]));
      } else {
        forceMagnitudes[j] = 0.0f; // Handle edge case for zero distance
      }
    }

    // Apply forces and update position/velocity for each planet
    math::FourVec2f forces = normalizedU * forceMagnitudes;
    for (int j = 0; j < 4; ++j) {
      planets_[i + j].body().ApplyForce(math::Vec2f(forces.x[j], forces.y[j]));
      planets_[i + j].body().Update(delta_time);
    }
  }

  // Handle any remaining planets that weren't in a set of four
  for (std::size_t i = simdSize; i < planets_.size(); ++i) {
    math::Vec2f u = star_.position() - planets_[i].position();
    const crackitos_physics::commons::fp r = u.Magnitude();
    if (r > 0) {
      const crackitos_physics::commons::fp force_magnitude = kGravitationConstant_ * (planets_[i].body().
          mass() * star_mass_ / (r * r));
      const math::Vec2f force = force_magnitude * u.Normalized();
      planets_[i].body().ApplyForce(force);
    }
    planets_[i].body().Update(delta_time);
  }
}

void PlanetSystem::SpawnPlanets(SDL_Color colour) {
  math::Vec2i mouse_pos;
  SDL_GetMouseState(&mouse_pos.x, &mouse_pos.y);
  const auto mouse_pos_f = math::Vec2f(static_cast<crackitos_physics::commons::fp>(mouse_pos.x),
                                       static_cast<crackitos_physics::commons::fp>(mouse_pos.y));

  auto distance = distance::Pixel((mouse_pos_f - star_.position()).Magnitude());
  auto minimum_distance = distance::Pixel(30);

  if (constexpr crackitos_physics::commons::fp minimum_range = 30;
      distance::Convert<distance::Pixel, distance::Meter>(distance).value
          > distance::Convert<distance::Pixel, distance::Meter>(minimum_distance).value) {
    const math::Vec2f random_pos(random::Range(mouse_pos_f.x - minimum_range, mouse_pos_f.x + minimum_range),
                                 random::Range(mouse_pos_f.y - minimum_range, mouse_pos_f.y + minimum_range));

    const crackitos_physics::commons::fp radius = random::Range(5.f, 20.f);
    const uint8_t alpha = random::Range(10, 255);
    colour.a = alpha;

    CreatePlanet(random_pos, radius, colour);
  }
}
} // namespace samples
