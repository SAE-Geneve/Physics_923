#ifndef PHYSICS_SAMPLES_GAME_ENGINE_H_
#define PHYSICS_SAMPLES_GAME_ENGINE_H_

#include "collision_system.h"
#include "friction_system.h"
#include "graphics_manager.h"
#include "imgui_interface.h"
#include "planet_system.h"
#include "trigger_system.h"

namespace crackitos_physics::samples
{
    enum class SystemScene
    {
        PlanetSystemScene,
        TriggerSystemScene,
        CollisionSystemScene,
        FrictionSystemScene
    };

    class GameEngine
    {
    private:
        SystemScene selected_scene_ = SystemScene::PlanetSystemScene;
        bool is_running_;

        Display display_{};
        crackitos_core::timer::Timer timer_{};
        GraphicsManager graphics_manager_{};
        PlanetSystem planet_system_{};
        TriggerSystem trigger_system_{};
        CollisionSystem collision_system_{};
        FrictionSystem friction_system_{};

        ImGuiInterface imgui_interface_{};

        void HandleEvents();
        void RenderQuadtree(SDL_Renderer* renderer, physics::Quadtree& quadtree);

    public:
        GameEngine();
        ~GameEngine();

        void ChangeScene(SystemScene new_sample);

        void Run();
    };
} // namespace samples
#endif // PHYSICS_SAMPLES_GAME_ENGINE_H_
