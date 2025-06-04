//
//	ShapeBox.h
//
#pragma once
#include "ShapeBase.h"

/*
====================================================
ShapeBox
====================================================
*/
class ShapeBox : public Shape {
public:
	explicit ShapeBox( const Vec3 * pts, const int num ) {
		Build( pts, num );
	}
	void Build( const Vec3 * pts, const int num );

	Vec3 Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const override;

	Mat3 InertiaTensor() const override;

	Bounds GetBounds( const Vec3 & pos, const Quat & orient ) const override;
	Bounds GetBounds() const override { return m_bounds; }

	float FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const override;

	shapeType_t GetType() const override { return SHAPE_BOX; }

	virtual std::optional<PointArrayAccessor> GetPointData() override;

	float GetLength() const;
	float GetWidth() const;
	float GetHeight() const;

	// 获取物体所有用于SAT测试的轴在局部空间下的向量，返回物体对应分离轴的数量
	// 对于不可使用SAT测试的物体返回-1
	virtual int GetSeperateAxis(std::vector<Vec3>& outAxis);

public:
	
	Vec3   m_pts[8];
	Bounds m_bounds;
};