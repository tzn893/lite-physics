#include "ShapePlane.h"
#include "Math/Helpers.h"

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
    Vec3 pt0 = Vec3( m_width, m_height, 0);
    Vec3 pt1 = Vec3(-m_width, m_height, 0);
    Vec3 pt2 = Vec3( m_width,-m_height, 0);
    Vec3 pt3 = Vec3(-m_width,-m_height, 0);


    pt0 = orient.RotatePoint(pt0) + pos;
    pt1 = orient.RotatePoint(pt1) + pos;
    pt2 = orient.RotatePoint(pt2) + pos;
    pt3 = orient.RotatePoint(pt3) + pos;

	Bounds tmp;
    tmp.maxs = Max(pt0, Max(pt1, Max(pt2, pt3)));
    tmp.mins = Min(pt0, Min(pt1, Min(pt2, pt3)));

    return tmp;
}

Bounds ShapePlane::GetBounds() const
{
    Bounds tmp;
	tmp.maxs = Vec3( m_width, m_height, 0) * 1.0f/2.0f;
    tmp.mins = Vec3(-m_width,-m_height, 0) * 1.0f/2.0f;

    return tmp;
}

Vec2 ShapePlane::GetExtent() const
{
    return Vec2(m_width, m_height);
}

Vec3 ShapePlane::GetNormalLocalSpace()
{
    return Vec3(0, 0, 1.0f);
}

Vec3 ShapePlane::GetNormalWorldSpace(Quat ori)
{
    return (ori.ToMat3().Transpose().Inverse() * GetNormalLocalSpace()).Normalize();
}
