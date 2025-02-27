#include "four_vec4.h"

#include <xmmintrin.h>

namespace physics923::math
{
    // Operator+ with SIMD intrinsics
    template <>
    FourVec4f FourVec4<physics923::commons::fp>::operator+(const FourVec4<physics923::commons::fp>& other) const
    {
        FourVec4<physics923::commons::fp> result;
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());
        __m128 z1 = _mm_loadu_ps(z.data());
        __m128 w1 = _mm_loadu_ps(w.data());

        __m128 x2 = _mm_loadu_ps(other.x.data());
        __m128 y2 = _mm_loadu_ps(other.y.data());
        __m128 z2 = _mm_loadu_ps(other.z.data());
        __m128 w2 = _mm_loadu_ps(other.w.data());

        __m128 x_res = _mm_add_ps(x1, x2);
        __m128 y_res = _mm_add_ps(y1, y2);
        __m128 z_res = _mm_add_ps(z1, z2);
        __m128 w_res = _mm_add_ps(w1, w2);

        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);
        _mm_storeu_ps(result.z.data(), z_res);
        _mm_storeu_ps(result.w.data(), w_res);

        return result;
    }

    // Operator- with SIMD intrinsics
    template <>
    FourVec4f FourVec4<physics923::commons::fp>::operator-(const FourVec4<physics923::commons::fp>& other) const
    {
        FourVec4<physics923::commons::fp> result;
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());
        __m128 z1 = _mm_loadu_ps(z.data());
        __m128 w1 = _mm_loadu_ps(w.data());

        __m128 x2 = _mm_loadu_ps(other.x.data());
        __m128 y2 = _mm_loadu_ps(other.y.data());
        __m128 z2 = _mm_loadu_ps(other.z.data());
        __m128 w2 = _mm_loadu_ps(other.w.data());

        __m128 x_res = _mm_sub_ps(x1, x2);
        __m128 y_res = _mm_sub_ps(y1, y2);
        __m128 z_res = _mm_sub_ps(z1, z2);
        __m128 w_res = _mm_sub_ps(w1, w2);

        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);
        _mm_storeu_ps(result.z.data(), z_res);
        _mm_storeu_ps(result.w.data(), w_res);

        return result;
    }

    // Negation operator
    template <>
    FourVec4f FourVec4<physics923::commons::fp>::operator-() const
    {
        FourVec4<physics923::commons::fp> result;
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());
        __m128 z1 = _mm_loadu_ps(z.data());
        __m128 w1 = _mm_loadu_ps(w.data());

        __m128 neg_one = _mm_set1_ps(-1.0f);

        __m128 x_res = _mm_mul_ps(x1, neg_one);
        __m128 y_res = _mm_mul_ps(y1, neg_one);
        __m128 z_res = _mm_mul_ps(z1, neg_one);
        __m128 w_res = _mm_mul_ps(w1, neg_one);

        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);
        _mm_storeu_ps(result.z.data(), z_res);
        _mm_storeu_ps(result.w.data(), w_res);

        return result;
    }

    // Scalar multiplication
    template <>
    FourVec4f FourVec4<physics923::commons::fp>::operator*(const physics923::commons::fp scalar) const
    {
        FourVec4<physics923::commons::fp> result;
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());
        __m128 z1 = _mm_loadu_ps(z.data());
        __m128 w1 = _mm_loadu_ps(w.data());

        __m128 scalar_reg = _mm_set1_ps(scalar);
        __m128 x_res = _mm_mul_ps(x1, scalar_reg);
        __m128 y_res = _mm_mul_ps(y1, scalar_reg);
        __m128 z_res = _mm_mul_ps(z1, scalar_reg);
        __m128 w_res = _mm_mul_ps(w1, scalar_reg);

        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);
        _mm_storeu_ps(result.z.data(), z_res);
        _mm_storeu_ps(result.w.data(), w_res);

        return result;
    }

    // Scalar division
    template <>
    FourVec4f FourVec4<physics923::commons::fp>::operator/(const physics923::commons::fp scalar) const
    {
        FourVec4<physics923::commons::fp> result;
        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());
        __m128 z1 = _mm_loadu_ps(z.data());
        __m128 w1 = _mm_loadu_ps(w.data());

        __m128 scalar_reg = _mm_set1_ps(scalar);
        __m128 x_res = _mm_div_ps(x1, scalar_reg);
        __m128 y_res = _mm_div_ps(y1, scalar_reg);
        __m128 z_res = _mm_div_ps(z1, scalar_reg);
        __m128 w_res = _mm_div_ps(w1, scalar_reg);

        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);
        _mm_storeu_ps(result.z.data(), z_res);
        _mm_storeu_ps(result.w.data(), w_res);

        return result;
    }

    // Dot product
    template <>
    std::array<physics923::commons::fp, 4> FourVec4<physics923::commons::fp>::Dot(const FourVec4f& other) const
    {
        std::array<physics923::commons::fp, 4> result;

        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());
        __m128 z1 = _mm_loadu_ps(z.data());
        __m128 w1 = _mm_loadu_ps(w.data());

        __m128 x2 = _mm_loadu_ps(other.x.data());
        __m128 y2 = _mm_loadu_ps(other.y.data());
        __m128 z2 = _mm_loadu_ps(other.z.data());
        __m128 w2 = _mm_loadu_ps(other.w.data());

        __m128 x_mul = _mm_mul_ps(x1, x2);
        __m128 y_mul = _mm_mul_ps(y1, y2);
        __m128 z_mul = _mm_mul_ps(z1, z2);
        __m128 w_mul = _mm_mul_ps(w1, w2);

        __m128 dot_res = _mm_add_ps(_mm_add_ps(_mm_add_ps(x_mul, y_mul), z_mul), w_mul);

        _mm_storeu_ps(result.data(), dot_res);

        return result;
    }

    // Square magnitude
    template <>
    std::array<physics923::commons::fp, 4> FourVec4<physics923::commons::fp>::SquareMagnitude() const
    {
        std::array<physics923::commons::fp, 4> result;

        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());
        __m128 z1 = _mm_loadu_ps(z.data());
        __m128 w1 = _mm_loadu_ps(w.data());

        __m128 x_sq = _mm_mul_ps(x1, x1);
        __m128 y_sq = _mm_mul_ps(y1, y1);
        __m128 z_sq = _mm_mul_ps(z1, z1);
        __m128 w_sq = _mm_mul_ps(w1, w1);

        __m128 square_magnitude = _mm_add_ps(_mm_add_ps(_mm_add_ps(x_sq, y_sq), z_sq), w_sq);

        _mm_storeu_ps(result.data(), square_magnitude);

        return result;
    }

    // Magnitude
    template <>
    std::array<physics923::commons::fp, 4> FourVec4<physics923::commons::fp>::Magnitude() const
    {
        std::array<physics923::commons::fp, 4> squared_magnitude = SquareMagnitude();
        __m128 squared_magnitude_ps = _mm_loadu_ps(squared_magnitude.data());
        __m128 magnitude_ps = _mm_sqrt_ps(squared_magnitude_ps);

        std::array<physics923::commons::fp, 4> result;
        _mm_storeu_ps(result.data(), magnitude_ps);

        return result;
    }

    // Normalize
    template <>
    FourVec4f FourVec4<physics923::commons::fp>::Normalize() const
    {
        FourVec4<physics923::commons::fp> result;
        std::array<physics923::commons::fp, 4> magnitude = Magnitude();

        __m128 x1 = _mm_loadu_ps(x.data());
        __m128 y1 = _mm_loadu_ps(y.data());
        __m128 z1 = _mm_loadu_ps(z.data());
        __m128 w1 = _mm_loadu_ps(w.data());

        __m128 magnitude_ps = _mm_loadu_ps(magnitude.data());

        __m128 x_res = _mm_div_ps(x1, magnitude_ps);
        __m128 y_res = _mm_div_ps(y1, magnitude_ps);
        __m128 z_res = _mm_div_ps(z1, magnitude_ps);
        __m128 w_res = _mm_div_ps(w1, magnitude_ps);

        _mm_storeu_ps(result.x.data(), x_res);
        _mm_storeu_ps(result.y.data(), y_res);
        _mm_storeu_ps(result.z.data(), z_res);
        _mm_storeu_ps(result.w.data(), w_res);

        return result;
    }

} // namespace math
