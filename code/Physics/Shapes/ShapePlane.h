#pragma once 
#include "Physics/Shapes/ShapeBase.h"

class ShapePlane : public Shape {
public:
	explicit ShapePlane(float width, float height);

	Vec3 Support(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias) const override;

	Mat3 InertiaTensor() const override;

	Bounds GetBounds(const Vec3& pos, const Quat& orient) const override;
	Bounds GetBounds() const override;

	shapeType_t GetType() const override { return SHAPE_PLANE; }

	Vec2 GetExtent() const;

	Vec3 GetNormalLocalSpace();
	Vec3 GetNormalWorldSpace(Quat ori);

public:
	float m_width, m_height;

	// plane的support函数类似一个很薄的box
	Vec3 m_points[8];
};
