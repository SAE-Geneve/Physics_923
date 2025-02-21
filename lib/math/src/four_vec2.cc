#include "four_vec2.h"

#include <xmmintrin.h>

namespace physics923::math
{
    //Operator+ with SIMD intrinsics
    template <>
    FourVec2f FourVec2<float>::operator+(const FourVec2<float>& other) const
    {
        FourVec2<float> result;
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 x2 = _mm_loadu_ps(other.x.data());
        __m128 y1 = _mm_loadu_ps(y.data());
        __m128 y2 = _mm_loadu_ps(other.y.data());

        __m128 x_res = _mm_add_ps(x1, x2);
        __m128 y_res = _mm_add_ps(y1, y2);

        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);

        return result;
    }

    //Operator- with SIMD intrinsics
    template <>
    FourVec2f FourVec2<float>::operator-(const FourVec2<float>& other) const
    {
        FourVec2<float> result;
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 x2 = _mm_loadu_ps(other.x.data());
        __m128 y1 = _mm_loadu_ps(y.data());
        __m128 y2 = _mm_loadu_ps(other.y.data());

        __m128 x_res = _mm_sub_ps(x1, x2);
        __m128 y_res = _mm_sub_ps(y1, y2);

        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);

        return result;
    }

    template <>
    FourVec2f FourVec2<float>::operator-() const
    {
        FourVec2<float> result;
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());

        // Create a SIMD register with -1.0f in all four slots
        __m128 neg_one = _mm_set1_ps(-1.0f);

        __m128 x_res = _mm_mul_ps(x1, neg_one);
        __m128 y_res = _mm_mul_ps(y1, neg_one);

        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);

        return result;
    }

    template <>
    FourVec2f FourVec2<float>::operator*(const float scalar) const
    {
        FourVec2<float> result;
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());

        __m128 scalar_reg = _mm_set1_ps(scalar);
        __m128 x_res = _mm_mul_ps(x1, scalar_reg);
        __m128 y_res = _mm_mul_ps(y1, scalar_reg);

        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);

        return result;
    }

    template <>
    FourVec2f FourVec2<float>::operator*(const std::array<float, 4> scalars) const
    {
        FourVec2<float> result;

        // Load x and y components of the current FourVec2f object
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());

        // Load scalar values into SIMD register
        __m128 scalar_reg = _mm_loadu_ps(scalars.data());

        // Perform element-wise multiplication
        __m128 x_res = _mm_mul_ps(x1, scalar_reg);
        __m128 y_res = _mm_mul_ps(y1, scalar_reg);

        // Store results back into the result's x and y arrays
        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);

        return result;
    }

    template <>
    FourVec2f FourVec2<float>::operator/(const float scalar) const
    {
        FourVec2<float> result;
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());

        __m128 scalar_reg = _mm_set1_ps(scalar);
        __m128 x_res = _mm_div_ps(x1, scalar_reg);
        __m128 y_res = _mm_div_ps(y1, scalar_reg);

        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);

        return result;
    }

    template <>
    std::array<float, 4> FourVec2<float>::Dot(const FourVec2f& other) const
    {
        std::array<float, 4> result;

        // Load the x and y components of the two vectors
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 x2 = _mm_loadu_ps(other.x.data());
        __m128 y1 = _mm_loadu_ps(y.data());
        __m128 y2 = _mm_loadu_ps(other.y.data());

        // Element-wise multiplication
        __m128 x_mul = _mm_mul_ps(x1, x2);
        __m128 y_mul = _mm_mul_ps(y1, y2);

        // Sum the corresponding x and y products
        __m128 dot_res = _mm_add_ps(x_mul, y_mul);

        // Store the result in the return array
        _mm_storeu_ps(result.data(), dot_res);

        return result;
    }

    template <>
std::array<float, 4> FourVec2<float>::SquareMagnitude() const
    {
        std::array<float, 4> result;

        // Load the x and y components
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());

        // Square the components
        __m128 x_sq = _mm_mul_ps(x1, x1);
        __m128 y_sq = _mm_mul_ps(y1, y1);

        // Sum the squares
        __m128 square_magnitude = _mm_add_ps(x_sq, y_sq);

        // Store the result
        _mm_storeu_ps(result.data(), square_magnitude);

        return result;
    }


    template <>
    std::array<float, 4> FourVec2<float>::Magnitude() const
    {
        // Call SquareMagnitude to get the squared magnitudes
        std::array<float, 4> squared_magnitude = SquareMagnitude();

        // Compute the square root of each element in the squared magnitudes
        __m128 squared_magnitude_ps = _mm_loadu_ps(squared_magnitude.data());
        __m128 magnitude_ps = _mm_sqrt_ps(squared_magnitude_ps);

        // Store the result in an array
        std::array<float, 4> result;
        _mm_storeu_ps(result.data(), magnitude_ps);

        return result;
    }


    template <>
    FourVec2f FourVec2<float>::Normalize() const
    {
        FourVec2<float> result;

        // Call Magnitude to get the magnitudes of the vector elements
        std::array<float, 4> magnitude = Magnitude();

        // Load the x and y components
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());

        // Load the magnitude values
        __m128 magnitude_ps = _mm_loadu_ps(magnitude.data());

        // Normalize by dividing x and y by the magnitude
        __m128 x_res = _mm_div_ps(x1, magnitude_ps);
        __m128 y_res = _mm_div_ps(y1, magnitude_ps);

        // Store the result
        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);

        return result;
    }

} //namespace math
