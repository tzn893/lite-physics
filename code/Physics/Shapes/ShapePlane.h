#pragma once 
#include "Physics/Shapes/ShapeBase.h"

class ShapePlane : public Shape {
public:
	explicit ShapePlane(float width, float height) : m_width(width), m_height(height) {
		m_centerOfMass.Zero();
	}

	Vec3 Support(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias) const override;

	Mat3 InertiaTensor() const override;

	Bounds GetBounds(const Vec3& pos, const Quat& orient) const override;
	Bounds GetBounds() const override;

	shapeType_t GetType() const override { return SHAPE_PLANE; }

	Vec2 GetExtent() const;

public:
	float m_width, m_height;
};
