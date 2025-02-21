#ifndef PHYSICS_923_API_GAME_ENGINE_H_
#define PHYSICS_923_API_GAME_ENGINE_H_

#include "collision_system.h"
#include "display.h"
#include "friction_system.h"
#include "graphics_manager.h"
#include "imgui_interface.h"
#include "planet_system.h"
#include "timer.h"
#include "trigger_system.h"
namespace physics923
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

    Display* display_;
    timer::Timer* timer_;
    GraphicsManager* graphics_manager_;
    PlanetSystem* planet_system_;
    TriggerSystem* trigger_system_;
    CollisionSystem* collision_system_;
    FrictionSystem* friction_system_;

    ImGuiInterface* imgui_interface_;

    void HandleEvents();

public:
    GameEngine();
    ~GameEngine();

    void ChangeScene(SystemScene new_sample);

    void Run();
};
}
#endif // PHYSICS_923_API_GAME_ENGINE_H_
