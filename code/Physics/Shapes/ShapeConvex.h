//
//	ShapeConvex.h
//
#pragma once
#include "ShapeBase.h"

struct ConvexTriangle {
	int a;
	int b;
	int c;
};

struct ConvexEdge {
	int a;
	int b;

	ConvexEdge() { a = 0, b = 0; }
	ConvexEdge(int a, int b);

	bool operator == ( const ConvexEdge & rhs ) const {
		return ( ( a == rhs.a && b == rhs.b ) || ( a == rhs.b && b == rhs.a ) );
	}
};

template <>
struct std::hash<ConvexEdge> {
	std::size_t operator()(const ConvexEdge& e) const {
		// 简单哈希组合方式（可使用 Boost 中的 hash_combine 替代）
		return std::hash<int>()(e.a) ^ (std::hash<int>()(e.b) << 1);
	}
};


// 存储各个面相邻面的索引
struct FaceAdjFaces
{
	int faceAdjCount = 0;
	int faceIndex[6];
};

// 存储各个边相邻面的索引
struct EdgeAdjFaces
{
	int faceIndex[2];
};


void BuildConvexHull(const std::vector< Vec3 > & verts, std::vector< Vec3 > & hullPts, std::vector< ConvexTriangle > & hullTris, std::vector<ConvexEdge>& hullEdges, std::vector<FaceAdjFaces>& hullAdjFaces );

/*
====================================================
ShapeConvex
====================================================
*/
class ShapeConvexBase : public Shape
{
public:
	// 根据顶点索引获取凸包顶点世界空间坐标
	virtual Vec3 GetConvexVertex(int idx, Vec3 positionWS, Quat oriWS) = 0;
	
	// 找到世界空间下距离接触点最近的面
	virtual std::vector<Vec3> FindClosestFaceByNormal(Vec3 normalWS, Vec3 positionWS, Quat oriWS, Vec3& normal, int& faceIdx) = 0;
	// 找到世界空间下距离接触点方向最近的边
	virtual std::vector<Vec3> FindClosestEdgeByContact(Vec3 contactWS, Vec3 positionWS, Quat oriWS) = 0;

	virtual FaceAdjFaces FindAdjFaces(int faceIdx) = 0;

	virtual void GetFaceInfo(int faceIdx, Vec3 positionWS, Quat oriWS, Vec3& normal, Vec3& origin) = 0;
};


class ShapeConvex : public ShapeConvexBase {
public:
	explicit ShapeConvex( const Vec3 * pts, const int num ) {
		Build( pts, num );
	}
	void Build( const Vec3 * pts, const int num );

	Vec3 Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const override;

	Mat3 InertiaTensor() const override { return m_inertiaTensor; }

	Bounds GetBounds( const Vec3 & pos, const Quat & orient ) const override;
	Bounds GetBounds() const override { return m_bounds; }

	float FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const override;

	virtual std::optional<PointArrayAccessor> GetPointData() override;

	shapeType_t GetType() const override { return SHAPE_CONVEX; }

	// 根据顶点索引获取凸包顶点世界空间坐标
	virtual Vec3 GetConvexVertex(int idx, Vec3 positionWS, Quat oriWS) override;

	// 找到世界空间下距离法线方向最近的面
	virtual std::vector<Vec3> FindClosestFaceByNormal(Vec3 normalWS, Vec3 positionWS, Quat oriWS, Vec3& normal, int& faceIdx) override;
	// 找到世界空间下距离法线方向最近的边
	virtual std::vector<Vec3> FindClosestEdgeByContact(Vec3 contactWS, Vec3 positionWS, Quat oriWS) override;

	const std::vector<Vec3>& GetVertices() const;

	virtual FaceAdjFaces FindAdjFaces(int faceIdx) override;

	virtual void GetFaceInfo(int faceIdx, Vec3 positionWS, Quat oriWS, Vec3& normal, Vec3& origin) override;

private:

	void GetTransformedTriangleVertices(ConvexTriangle tri, Vec3 positionWS, Quat oriWS, Vec3& p0, Vec3& p1, Vec3& p2);
	void GetTransformedEdgeVertices(ConvexEdge edge, Vec3 positionWS, Quat oriWS, Vec3& p0, Vec3& p1);

	std::vector<FaceAdjFaces> m_adjFaces;
	std::vector<ConvexEdge> m_edges;
	std::vector<ConvexTriangle> m_triangles;
	std::vector< Vec3 > m_points;
	Bounds m_bounds;
	Mat3 m_inertiaTensor;

	Vec3   m_coners[8];
};

