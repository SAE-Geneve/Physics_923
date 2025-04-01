#include "imgui_interface.h"

#include <imgui_impl_sdlrenderer2.h>

#include "game_engine.h"

namespace crackitos_physics::samples
{
    void ImGuiInterface::Initialize(Display* display, GameEngine* engine)
    {
        game_engine_ = engine;

        //Setup ImGUI context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        (void)io;

        //Setup Platform/Renderer bindings
        ImGui_ImplSDL2_InitForSDLRenderer(display->window(), display->renderer());
        ImGui_ImplSDLRenderer2_Init(display->renderer());
    }

    void ImGuiInterface::Update(bool& show_imgui)
    {
        //Start new ImGui frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        //Render ImGui window with a checkbox
        if (show_imgui)
        {
            // Set the position and size of the ImGui window
            ImVec2 window_pos = ImVec2(0, 0); // Set the position (x, y)
            ImVec2 window_size = ImVec2(300, 400); // Set the size (width, height)
            ImGui::SetNextWindowPos(window_pos, ImGuiCond_Once); // Position it once
            ImGui::SetNextWindowSize(window_size, ImGuiCond_Once); // Size it once

            // Create a window
            ImGui::Begin("Kuma Engine", nullptr,
                         ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_AlwaysAutoResize);

            // Display FPS at the top
            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);


            ImGui::Separator();

            // Scene Selector Combo Box
            const char* scenes[] = {"Planet System", "Trigger System", "Collision System", "Friction System", "Testing System"};

            if (ImGui::BeginCombo("Select Scene", scenes[current_scene_]))
            {
                for (int n = 0; n < IM_ARRAYSIZE(scenes); n++)
                {
                    const bool is_selected = (current_scene_ == n);
                    if (ImGui::Selectable(scenes[n], is_selected))
                    {
                        current_scene_ = n;
                        speed_multiplier_ = 1.0f;
                        if (game_engine_)
                        {
                            game_engine_->ChangeScene(static_cast<SystemScene>(current_scene_));
                        }
                    }
                    if (is_selected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            // Reset Scene Button
            if (ImGui::Button("Reset Scene"))
            {
                speed_multiplier_ = 1.0f;
                if (game_engine_)
                {
                    game_engine_->ChangeScene(static_cast<SystemScene>(current_scene_));
                }
            }
            ImGui::Separator();

            // Scene-specific options
            switch (static_cast<SystemScene>(current_scene_))
            {
            case SystemScene::PlanetSystemScene: // Planet System
                {
                    ImGui::PushTextWrapPos();
                    ImGui::Text("Left Click: Toggle Spawner");
                    ImGui::PopTextWrapPos();

                    ImGui::Separator();

                    static ImVec4 planetColor = ImVec4(1.0f, 13.f / 255.f, 132.f / 255.f, 1.0f); // Default color
                    if (ImGui::ColorPicker3("Planet Color",
                                            reinterpret_cast<crackitos_physics::commons::fp*>(&planetColor)))
                    {
                        // Convert the ImVec3 color (0-1 range) to SDL_Color (0-255 range)
                        SDL_Color sdlColor;
                        sdlColor.r = static_cast<Uint8>(planetColor.x * 255);
                        sdlColor.g = static_cast<Uint8>(planetColor.y * 255);
                        sdlColor.b = static_cast<Uint8>(planetColor.z * 255);
                        sdlColor.a = 255; // Full opacity, since ColorPicker3 does not have an alpha channel
                        planets_colour_ = sdlColor;
                    }

                    ImGui::SliderFloat("Speed Mult", &speed_multiplier_, 0.0f, 10.0f);
                    break;
                }
            // case SystemScene::TriggerSystemScene: // Trigger System
            //     {
            //         ImGui::Checkbox("Show Quadtree", &show_quadtree_);
            //
            //         ImGui::SliderFloat("Speed Mult", &speed_multiplier_, 0.0f, 10.0f);
            //         break;
            //     }
            // case SystemScene::CollisionSystemScene: // Collision System
            //     {
            //         ImGui::Checkbox("Show Quadtree", &show_quadtree_);
            //
            //         ImGui::SliderFloat("Speed Mult", &speed_multiplier_, 0.0f, 10.0f);
            //         break;
            //     }
            case SystemScene::FrictionSystemScene: // Friction System
                {
                    ImGui::PushTextWrapPos();
                    ImGui::Text("Left Click: Spawn Circle");
                    ImGui::Text("Right Click: Spawn AABB");
                    ImGui::PopTextWrapPos();

                    ImGui::Separator();

                    ImGui::Checkbox("Show Quadtree", &show_quadtree_);

                    ImGui::SliderFloat("Speed Mult", &speed_multiplier_, 0.0f, 2.0f);
                    break;
                }
            case SystemScene::TestingSystemScene: // Friction System
                {
                    ImGui::PushTextWrapPos();
                    ImGui::Text("Left Click: Spawn Circle");
                    ImGui::Text("Right Click: Spawn AABB");
                    ImGui::PopTextWrapPos();

                    ImGui::Separator();

                    ImGui::Checkbox("Show Quadtree", &show_quadtree_);

                    ImGui::SliderFloat("Speed Mult", &speed_multiplier_, 0.0f, 10.0f);
                    break;
                }
            default:
                {
                    ImGui::Text("No options available for this scene.");
                    break;
                }
            }

            // End the window
            ImGui::End();
        }
    }

    void ImGuiInterface::Render(SDL_Renderer* renderer)
    {
        //Render ImGui on top of SDL renderer
        ImGui::Render();
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
    }

    void ImGuiInterface::PassEvents(SDL_Event& event)
    {
        ImGui_ImplSDL2_ProcessEvent(&event);
    }

    ImGuiInterface::~ImGuiInterface()
    {
        //Cleanup ImGui
        ImGui_ImplSDLRenderer2_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();
    }
} // namespace samples
