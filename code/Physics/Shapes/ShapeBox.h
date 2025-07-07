//
//	ShapeBox.h
//
#pragma once
#include "ShapeBase.h"
#include "ShapeConvex.h"

/*
====================================================
ShapeBox
====================================================
*/
class ShapeBox : public ShapeConvexBase {
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

	// 根据顶点索引获取凸包顶点世界空间坐标
	virtual Vec3 GetConvexVertex(int idx, Vec3 positionWS, Quat oriWS) override;

	// 找到世界空间下距离法线方向最近的面
	virtual std::vector<Vec3> FindClosestFaceByNormal(Vec3 normalWS, Vec3 positionWS, Quat oriWS, Vec3& normal, int& faceIdx) override;
	// 找到世界空间下距离法线方向最近的边
	virtual std::vector<Vec3> FindClosestEdgeByContact(Vec3 contactWS, Vec3 positionWS, Quat oriWS) override;

	virtual FaceAdjFaces FindAdjFaces(int faceIdx) override;

	virtual void GetFaceInfo(int faceIdx, Vec3 positionWS, Quat oriWS, Vec3& normal, Vec3& origin) override;

private:
	
	Vec3   m_pts[8];
	Bounds m_bounds;
};