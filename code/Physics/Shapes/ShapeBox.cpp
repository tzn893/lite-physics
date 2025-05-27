//
//  Shapes.cpp
//
#include "ShapeBox.h"
#include "Math/Helpers.h"
/*
========================================================================================================

ShapeBox

========================================================================================================
*/

/*
====================================================
ShapeBox::Build
====================================================
*/
void ShapeBox::Build( const Vec3 * pts, const int num ) 
{
	for (int i = 0 ;i < num; i++)
	{
		m_bounds.Expand(pts[i]);
	}

	// 将质心挪到物体空间的原点
	Vec3 center = (m_bounds.maxs + m_bounds.mins) / 2.0f;
	m_bounds.maxs -= center;
	m_bounds.mins -= center;


	m_pts[0] = m_bounds.mins;
	m_pts[1] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z);
	m_pts[2] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.mins.z);
	m_pts[3] = Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.mins.z);
	m_pts[4] = Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.maxs.z);
	m_pts[5] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.maxs.z);
	m_pts[6] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.maxs.z);
	m_pts[7] = m_bounds.maxs;
}

/*
====================================================
ShapeBox::Support
====================================================
*/
Vec3 ShapeBox::Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const {
	Vec3 supportPt;
	
	// 找到距离最远的顶点
	Vec3 maxPt = orient.RotatePoint(m_pts[0]);
	float maxDist = dir.Dot(maxPt);
	for (int i = 1; i < 8; i++)
	{
		Vec3 pt = orient.RotatePoint(m_pts[i]);
		float dist = dir.Dot(pt);
		if (dist > maxDist)
		{
			maxDist = dist;
			maxPt = pt;
		}
	}

	return maxPt + maxPt.Dir() * bias + pos;
}

/*
====================================================
ShapeBox::InertiaTensor
====================================================
*/
Mat3 ShapeBox::InertiaTensor() const {
	Mat3 tensor;
	
	float x = GetLength();
	float y = GetWidth();
	float z = GetHeight();

	tensor.rows[0][0] = 1.0f / 12.0f * (y * y + z * z);
	tensor.rows[0][0] = 1.0f / 12.0f * (y * y + z * z);
	tensor.rows[1][1] = 1.0f / 12.0f * (x * x + z * z);
	tensor.rows[2][2] = 1.0f / 12.0f * (y * y + x * x);


	return tensor;
}

/*
====================================================
ShapeBox::GetBounds
====================================================
*/
Bounds ShapeBox::GetBounds( const Vec3 & pos, const Quat & orient ) const {
	Bounds bounds;

	for (int i = 0;i < 8;i++)
	{
		Vec3 pt = pos + orient.RotatePoint(m_pts[i]);
		bounds.Expand(pt);
	}

	return bounds;
}

/*
====================================================
ShapeBox::FastestLinearSpeed
====================================================
*/
float ShapeBox::FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const {
	
	float maxLinearSpeed = 0; 
	for (int i = 0;i < 8; i++)
	{
		Vec3 vel = angularVelocity.Cross(m_pts[i]);
		maxLinearSpeed = Max(vel.Dot(dir), maxLinearSpeed);
	}

	return maxLinearSpeed;
}


std::optional<PointArrayAccessor> ShapeBox::GetPointData()
{
	return PointArrayAccessor(m_pts, 8);
}

float ShapeBox::GetLength() const
{
	return m_bounds.maxs.x - m_bounds.mins.x;
}

float ShapeBox::GetWidth() const
{
	return m_bounds.maxs.y - m_bounds.mins.y;
}

float ShapeBox::GetHeight() const
{
	return m_bounds.maxs.z - m_bounds.mins.z;
}
