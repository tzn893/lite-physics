#include "ShapePlane.h"
#include "Math/Helpers.h"

ShapePlane::ShapePlane(float width, float height): m_width(width), m_height(height)
{
    // plane就是厚度很薄的box
    float thick = 5e-4f;

    m_centerOfMass = Vec3(0, 0, 0);
    
    m_points[0] = Vec3(width * 0.5f, height * 0.5f, thick * 0.5f);
    m_points[1] = Vec3(width *-0.5f, height * 0.5f, thick * 0.5f);
    m_points[2] = Vec3(width * 0.5f, height *-0.5f, thick * 0.5f);
    m_points[3] = Vec3(width *-0.5f, height *-0.5f, thick * 0.5f);
    m_points[4] = Vec3(width * 0.5f, height * 0.5f, thick *-0.5f);
    m_points[5] = Vec3(width *-0.5f, height * 0.5f, thick *-0.5f);
    m_points[6] = Vec3(width * 0.5f, height *-0.5f, thick *-0.5f);
    m_points[7] = Vec3(width *-0.5f, height *-0.5f, thick *-0.5f);
}

Vec3 ShapePlane::Support(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias) const
{
    Vec3 supportPt;

    // 找到距离最远的顶点
    Vec3 maxPt = orient.RotatePoint(m_points[0]);
    float maxDist = dir.Dot(maxPt);
    for (int i = 1; i < 8; i++)
    {
        Vec3 pt = orient.RotatePoint(m_points[i]);
        float dist = dir.Dot(pt);
        if (dist > maxDist)
        {
            maxDist = dist;
            maxPt = pt;
        }
    }

    return maxPt + maxPt.Dir() * bias + pos;
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
    Bounds bounds;

    for (int i = 0; i < 8; i++)
    {
        Vec3 pt = pos + orient.RotatePoint(m_points[i]);
        bounds.Expand(pt);
    }

    return bounds;
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
