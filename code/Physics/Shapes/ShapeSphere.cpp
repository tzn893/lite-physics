//
//  ShapeSphere.cpp
//
#include "ShapeSphere.h"

/*
========================================================================================================

ShapeSphere

========================================================================================================
*/

/*
====================================================
ShapeSphere::Support
====================================================
*/
Vec3 ShapeSphere::Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const {
	Vec3 supportPt;
	
	return  dir * (m_radius + bias);
}

/*
====================================================
ShapeSphere::InertiaTensor
====================================================
*/
Mat3 ShapeSphere::InertiaTensor() const {
	Mat3 tensor;
	
	tensor.rows[0][0] = 2.0f / 5.0f * m_radius * m_radius;
	tensor.rows[1][1] = tensor.rows[0][0];
	tensor.rows[2][2] = tensor.rows[0][0];

	return tensor;
}

/*
====================================================
ShapeSphere::GetBounds
====================================================
*/
Bounds ShapeSphere::GetBounds( const Vec3 & pos, const Quat & orient ) const {
	Bounds tmp;
	tmp.maxs = Vec3( m_radius, m_radius, m_radius) + pos;
	tmp.mins = Vec3(-m_radius,-m_radius,-m_radius) + pos;

	return tmp;
}

/*
====================================================
ShapeSphere::GetBounds
====================================================
*/
Bounds ShapeSphere::GetBounds() const {
	Bounds tmp;
	
	// TODO: Add code
	tmp.maxs = Vec3( m_radius, m_radius, m_radius);
	tmp.mins = Vec3(-m_radius,-m_radius,-m_radius);

	return tmp;
}

float ShapeSphere::GetRadius()
{
	return m_radius;
}
