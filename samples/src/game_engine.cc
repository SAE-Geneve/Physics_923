#include "game_engine.h"

#include <SDL_events.h>

#include "imgui_interface.h"
#include "random.h"

namespace crackitos_physics::samples
{
    GameEngine::GameEngine()
    {
        is_running_ = true;
        imgui_interface_.Initialize(&display_, this);
    }

    GameEngine::~GameEngine()
    = default;


    void GameEngine::ChangeScene(const SystemScene new_sample)
    {
        // Perform cleanup for the current scene
        switch (selected_scene_)
        {
        case SystemScene::PlanetSystemScene: // Planet System
            planet_system_.Clear(); // Hypothetical cleanup method
            break;
        case SystemScene::TriggerSystemScene: // Trigger System
            trigger_system_.Clear(); // Cleanup if needed
            break;
        case SystemScene::CollisionSystemScene: // Collision System
            collision_system_.Clear(); // Cleanup if needed
            break;
        case SystemScene::FrictionSystemScene: // Friction System
            friction_system_.Clear(); // Cleanup if needed
            break;
        case SystemScene::TestingSystemScene:
            testing_system_.Clear();
            break;
        default:
            break;
        }

        //Update to the new scene
        selected_scene_ = new_sample;

        // Initialize the new scene
        switch (selected_scene_)
        {
        case SystemScene::PlanetSystemScene: // Planet System
            planet_system_.Initialize();
            break;
        case SystemScene::TriggerSystemScene: // Trigger System
            trigger_system_.Initialize();
            break;
        case SystemScene::CollisionSystemScene: // Collision System
            collision_system_.Initialize();
            break;
        case SystemScene::FrictionSystemScene: // Friction System
            friction_system_.Initialize();
            break;
        case SystemScene::TestingSystemScene:
            testing_system_.Initialize();
        default: break;
        }
    }

    void GameEngine::HandleEvents()
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            // Pass events to ImGui for input handling
            imgui_interface_.PassEvents(event);

            if (event.type == SDL_QUIT)
            {
                is_running_ = false;
            }
            else if (event.type == SDL_KEYDOWN)
            {
                if (event.key.keysym.scancode == SDL_SCANCODE_ESCAPE)
                {
                    is_running_ = false;
                }
            }
            else if (event.type == SDL_MOUSEBUTTONDOWN && !ImGui::GetIO().WantCaptureMouse)
            {
                if (event.button.button == SDL_BUTTON_LEFT)
                {
                    if (selected_scene_ == SystemScene::PlanetSystemScene)
                    {
                        // PLANET SYSTEM:
                        planet_system_.ToggleSpawner();
                    }
                    if (selected_scene_ == SystemScene::FrictionSystemScene)
                    {
                        int mouse_x, mouse_y;
                        SDL_GetMouseState(&mouse_x, &mouse_y);
                        const auto mouse_pos = crackitos_core::math::Vec2f(static_cast<crackitos_core::commons::fp>(mouse_x),
                                                           static_cast<crackitos_core::commons::fp>(mouse_y));
                        friction_system_.SpawnShape(mouse_pos, crackitos_core::math::ShapeType::kCircle);
                    }
                    if (selected_scene_ == SystemScene::TestingSystemScene)
                    {
                        int mouse_x, mouse_y;
                        SDL_GetMouseState(&mouse_x, &mouse_y);
                        const auto mouse_pos = math::Vec2f(static_cast<commons::fp>(mouse_x),
                                                           static_cast<commons::fp>(mouse_y));
                        testing_system_.SpawnShape(mouse_pos, math::ShapeType::kCircle);
                    }
                }
                else if (event.button.button == SDL_BUTTON_RIGHT && !ImGui::GetIO().WantCaptureMouse)
                {
                    if (selected_scene_ == SystemScene::FrictionSystemScene)
                    {
                        int mouse_x, mouse_y;
                        SDL_GetMouseState(&mouse_x, &mouse_y);
                        const auto mouse_pos = crackitos_core::math::Vec2f(static_cast<crackitos_core::commons::fp>(mouse_x),
                                                           static_cast<crackitos_core::commons::fp>(mouse_y));
                        friction_system_.SpawnShape(mouse_pos, crackitos_core::math::ShapeType::kAABB);
                    }
                    if (selected_scene_ == SystemScene::TestingSystemScene)
                    {
                        int mouse_x, mouse_y;
                        SDL_GetMouseState(&mouse_x, &mouse_y);
                        const auto mouse_pos = math::Vec2f(static_cast<commons::fp>(mouse_x),
                                                           static_cast<commons::fp>(mouse_y));
                        testing_system_.SpawnShape(mouse_pos, math::ShapeType::kAABB);
                    }
                }
            }
            // else if (event.type == SDL_MOUSEBUTTONUP)
            // {
            //     if (event.button.button == SDL_BUTTON_LEFT)
            //     {
            //     }
            // }
        }
    }

    void GameEngine::RenderQuadtree(SDL_Renderer* renderer, physics::Quadtree& quadtree)
    {
        quadtree.UpdateBoundingBoxes();
        const std::vector<crackitos_core::math::AABB>& boxes = quadtree.GetBoundingBoxes(); // No copying

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        for (const auto& box : boxes)
        {
            SDL_Rect rect;
            rect.x = static_cast<int>(box.min_bound().x);
            rect.y = static_cast<int>(box.min_bound().y);
            rect.w = static_cast<int>(box.max_bound().x - box.min_bound().x);
            rect.h = static_cast<int>(box.max_bound().y - box.min_bound().y);

            SDL_RenderDrawRect(renderer, &rect);
        }
    }

    void GameEngine::Run()
    {
        ChangeScene(selected_scene_);
        //Begin():
        timer_.SetFixedDeltaTime(1.0f / 60.0f); // Fixed step of 60 FPS (0.0167 sec)

        while (is_running_)
        {
            // Handle events
            HandleEvents();
            imgui_interface_.Update(is_running_);

            // Update the timer
            timer_.Tick();
            crackitos_core::commons::fp delta_time = timer_.DeltaTime();


            while (timer_.FixedDeltaTimeStep())
            {
                // Update all systems with the fixed time step
                if (selected_scene_ == SystemScene::PlanetSystemScene)
                {
                    planet_system_.Update(timer_.FixedDeltaTime() * imgui_interface_.speed_multiplier() * 1000.0f,
                                          imgui_interface_.planets_colour());
                }
                else if (selected_scene_ == SystemScene::TriggerSystemScene)
                {
                    trigger_system_.Update(timer_.FixedDeltaTime() * imgui_interface_.speed_multiplier());
                }
                else if (selected_scene_ == SystemScene::CollisionSystemScene)
                {
                    collision_system_.Update(timer_.FixedDeltaTime() * imgui_interface_.speed_multiplier());
                }
                else if (selected_scene_ == SystemScene::FrictionSystemScene)
                {
                    friction_system_.Update(timer_.FixedDeltaTime() * imgui_interface_.speed_multiplier() * 4.f);
                }
                else if (selected_scene_ == SystemScene::TestingSystemScene)
                {
                    testing_system_.Update(timer_.FixedDeltaTime() * imgui_interface_.speed_multiplier());
                }
            }

            // Render
            display_.Clear();
            graphics_manager_.Clear();

            // Render all systems based on the current state
            if (selected_scene_ == SystemScene::PlanetSystemScene)
            {
                graphics_manager_.CreateCircle(planet_system_.star()->position(), 10.f, SDL_Color(255, 255, 255, 150),
                                               false);
                for (const auto& p : planet_system_.planets())
                {
                    graphics_manager_.CreateCircle(p.position(), p.radius(), p.color(), false);
                }
            }
            else if (selected_scene_ == SystemScene::TriggerSystemScene)
            {
                for (const auto& obj : trigger_system_.trigger_objects())
                {
                    const auto& collider = trigger_system_.physics_world().GetCollider(obj.collider);
                    const auto& body = trigger_system_.physics_world().GetBody(obj.body);

                    switch (collider.GetShapeType())
                    {
                    case crackitos_core::math::ShapeType::kAABB:
                        graphics_manager_.CreateAABB(
                            collider.GetBoundingBox().min_bound(),
                            collider.GetBoundingBox().max_bound(),
                            obj.color, true);
                        break;

                    case crackitos_core::math::ShapeType::kCircle:
                        graphics_manager_.CreateCircle(
                            body.position(),
                            collider.GetBoundingBox().half_size_vec().x, // Radius
                            obj.color, false);
                        break;

                    default:
                        break;
                    }
                }

                if (imgui_interface_.show_quadtree())
                {
                    RenderQuadtree(display_.renderer(), trigger_system_.physics_world().quadtree());
                }
            }
            else if (selected_scene_ == SystemScene::CollisionSystemScene)
            {
                for (const auto& obj : collision_system_.collision_objects())
                {
                    const auto& collider = collision_system_.physics_world().GetCollider(obj.collider);
                    const auto& body = collision_system_.physics_world().GetBody(obj.body);

                    switch (collider.GetShapeType())
                    {
                    case crackitos_core::math::ShapeType::kAABB:
                        graphics_manager_.CreateAABB(
                            collider.GetBoundingBox().min_bound(),
                            collider.GetBoundingBox().max_bound(),
                            obj.color, true);
                        break;

                    case crackitos_core::math::ShapeType::kCircle:
                        graphics_manager_.CreateCircle(
                            body.position(),
                            collider.GetBoundingBox().half_size_vec().x, // Radius
                            obj.color, false);
                        break;

                    default:
                        break;
                    }
                }
                if (imgui_interface_.show_quadtree())
                {
                    RenderQuadtree(display_.renderer(), collision_system_.physics_world().quadtree());
                }
            }
            else if (selected_scene_ == SystemScene::FrictionSystemScene)
            {
                for (const auto& obj : friction_system_.friction_objects())
                {
                    const auto& collider = friction_system_.physics_world().GetCollider(obj.collider);
                    const auto& body = friction_system_.physics_world().GetBody(obj.body);

                    switch (collider.GetShapeType())
                    {
                    case crackitos_core::math::ShapeType::kAABB:
                        graphics_manager_.CreateAABB(
                            collider.GetBoundingBox().min_bound(),
                            collider.GetBoundingBox().max_bound(),
                            obj.color, true);
                        break;

                    case crackitos_core::math::ShapeType::kCircle:
                        graphics_manager_.CreateCircle(
                            body.position(),
                            collider.GetBoundingBox().half_size_vec().x, // Radius
                            obj.color, false);
                        break;

                    default:
                        break;
                    }
                }

                if (imgui_interface_.show_quadtree())
                {
                    RenderQuadtree(display_.renderer(), friction_system_.physics_world().quadtree());
                }
            }
            else if (selected_scene_ == SystemScene::TestingSystemScene)
            {
                for (const auto& obj : testing_system_.testing_objects())
                {
                    const auto& collider = testing_system_.physics_world().GetCollider(obj.collider);
                    const auto& body = testing_system_.physics_world().GetBody(obj.body);

                    switch (collider.GetShapeType())
                    {
                    case math::ShapeType::kAABB:
                        graphics_manager_.CreateAABB(
                            collider.GetBoundingBox().min_bound(),
                            collider.GetBoundingBox().max_bound(),
                            obj.color, true);
                        break;

                    case math::ShapeType::kCircle:
                        graphics_manager_.CreateCircle(
                            body.position(),
                            collider.GetBoundingBox().half_size_vec().x, // Radius
                            obj.color, false);
                        break;

                    default:
                        break;
                    }
                }

                if (imgui_interface_.show_quadtree())
                {
                    RenderQuadtree(display_.renderer(), testing_system_.physics_world().quadtree());
                }
            }


            // Render the graphics
            SDL_RenderGeometry(display_.renderer(),
                               nullptr,
                               graphics_manager_.vertices().data(),
                               static_cast<int>(graphics_manager_.vertices().size()),
                               graphics_manager_.indices().data(),
                               static_cast<int>(graphics_manager_.indices().size()));

            imgui_interface_.Render(display_.renderer());
            SDL_RenderPresent(display_.renderer());
        }
    } // End()
} // namespace samples
