#include "graphics_manager.h"

#include "commons.h"

namespace physics923
{
    void GraphicsManager::AddVertex(const math::Vec2f position, const SDL_Color color)
    {
        SDL_Vertex vertex;
        vertex.position.x = position.x;
        vertex.position.y = position.y;
        vertex.color = color;
        vertices_.push_back(vertex);
    }

    void GraphicsManager::Clear()
    {
        vertices_.clear();
        indices_.clear();
    }

    void GraphicsManager::CreateCircle(const math::Vec2f centre, const physics923::commons::fp radius, const SDL_Color color, const bool rotation)
    {
        //Track where the new circle's vertices start
        const size_t starting_index = vertices_.size();

        constexpr physics923::commons::fp angle_step = (2 * commons::Pi) / kCircleVertexCount;

        //Add the centre of the circle
        AddVertex(centre, SDL_Color{0, 0, 0, 0});

        {
            const physics923::commons::fp angle = static_cast<physics923::commons::fp>(0) * angle_step;
            const physics923::commons::fp x = centre.x + radius * std::cos(angle);
            const physics923::commons::fp y = centre.y + radius * std::sin(angle);
            if(rotation)
            {
                AddVertex(math::Vec2f{x, y}, SDL_Color{0, 0, 0, 255});
            }
            else
            {
                AddVertex(math::Vec2f{x, y}, SDL_Color{color.r, color.g, color.b, color.a});
            }
        }


        //Generate vertices
        for (size_t i = 1; i < kCircleVertexCount; i++)
        {
            const physics923::commons::fp angle = static_cast<physics923::commons::fp>(i) * angle_step;
            const physics923::commons::fp x = centre.x + radius * std::cos(angle);
            const physics923::commons::fp y = centre.y + radius * std::sin(angle);

            AddVertex(math::Vec2f{x, y}, color);
        }

        //Generate indices
        for (size_t i = 0; i < kCircleVertexCount - 1; i++)
        {
            indices_.push_back(static_cast<int>(starting_index));          //Center vertex
            indices_.push_back(static_cast<int>(starting_index + i + 1));  //Current outer vertex
            indices_.push_back(static_cast<int>(starting_index + i + 2));  //Next outer vertex
        }


        //Last triangle, closing the circle
        indices_.push_back(static_cast<int>(starting_index));
        indices_.push_back(static_cast<int>(starting_index + kCircleVertexCount));
        indices_.push_back(static_cast<int>(starting_index + 1));

    }

    void GraphicsManager::CreateAABB(const math::Vec2f min, const math::Vec2f max, const SDL_Color color, bool fill_status)
    {
        const size_t starting_index = vertices_.size();
        AddVertex(min, color);
        AddVertex(math::Vec2f{min.x, max.y}, color);
        AddVertex(max, color);
        AddVertex(math::Vec2f{max.x, min.y}, color);

        indices_.push_back(static_cast<int>(starting_index));
        indices_.push_back(static_cast<int>(starting_index + 1));
        indices_.push_back(static_cast<int>(starting_index + 2));

        indices_.push_back(static_cast<int>(starting_index));
        indices_.push_back(static_cast<int>(starting_index + 2));
        indices_.push_back(static_cast<int>(starting_index + 3));
    }

    void GraphicsManager::CreatePolygon(const std::vector<math::Vec2f>& points, const math::Vec2f center, const SDL_Color color, bool fill_status)
    {
        const size_t starting_index = vertices_.size();

        AddVertex(center, color);

        //Generate vertices
        for (const auto& point : points)
        {
            AddVertex(math::Vec2f{point.x, point.y}, color);
        }

        //Generate indices
        for (size_t i = 0; i < points.size() - 1; i++)
        {
            indices_.push_back(static_cast<int>(starting_index));          //Center vertex
            indices_.push_back(static_cast<int>(starting_index + i + 1));  //Current outer vertex
            indices_.push_back(static_cast<int>(starting_index + i + 2));  //Next outer vertex
        }


        //Last triangle, closing the shape
        indices_.push_back(static_cast<int>(starting_index));
        indices_.push_back(static_cast<int>(starting_index + points.size()));
        indices_.push_back(static_cast<int>(starting_index + 1));
    }
}
