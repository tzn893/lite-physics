//
//	ShapeBase.h
//
#pragma once
#include "../../Math/Vector.h"
#include "../../Math/Quat.h"
#include "../../Math/Matrix.h"
#include "../../Math/Bounds.h"
#include <vector>
#include <optional>

/*
====================================================
Shape
====================================================
*/
class PointArrayAccessor
{
public:
	PointArrayAccessor():data(nullptr),count(0){}
	PointArrayAccessor(const Vec3* data, size_t num) : data(data), count(num) {}
	PointArrayAccessor(const std::vector<Vec3>& data);

	Vec3 operator[](size_t idx) const;

private:
	const Vec3* data;
	size_t count;

};



class Shape {
public:
	virtual Mat3 InertiaTensor() const = 0;

	virtual Bounds GetBounds( const Vec3 & pos, const Quat & orient ) const = 0;
	virtual Bounds GetBounds() const = 0;

	virtual Vec3 GetCenterOfMass() const { return m_centerOfMass; }

	enum shapeType_t {
		SHAPE_SPHERE,
		SHAPE_BOX,
		SHAPE_CONVEX,
		SHAPE_PLANE,
		SHAPE_COUNT
	};
	virtual shapeType_t GetType() const = 0;

	virtual Vec3 Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const = 0;

	virtual float FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const { return 0.0f; }

	virtual std::optional<PointArrayAccessor> GetPointData() { return std::nullopt; }

protected:
	Vec3 m_centerOfMass;
};

using EShape = Shape::shapeType_t;