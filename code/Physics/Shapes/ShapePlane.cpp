#include "ShapePlane.h"

Vec3 ShapePlane::Support(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias) const
{
    return Vec3();
}

Mat3 ShapePlane::InertiaTensor() const
{
    Mat3 tensor;

    tensor.rows[0][0] = 1 / 12.f * m_height * m_height;
	tensor.rows[1][1] = 1 / 12.f * m_width * m_width;
    tensor.rows[2][2] = 1 / 12.f * (m_height * m_height + m_width * m_width);

    return tensor;
}

Bounds ShapePlane::GetBounds(const Vec3& pos, const Quat& orient) const
{
    return Bounds();
}

Bounds ShapePlane::GetBounds() const
{
    return Bounds();
}

Vec2 ShapePlane::GetExtent() const
{
    return Vec2(m_width, m_height);
}
