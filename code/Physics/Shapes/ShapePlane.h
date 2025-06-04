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

	// 获取物体所有用于SAT测试的轴在局部空间下的向量，返回物体对应分离轴的数量
	// 对于不可使用SAT测试的物体返回-1
	virtual int GetSeperateAxis(std::vector<Vec3>& outAxis);

private:
	float m_width, m_height;

	// plane的support函数类似一个很薄的box
	Vec3 m_points[8];
};
