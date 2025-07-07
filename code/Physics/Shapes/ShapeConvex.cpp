//
//  ShapeConvex.cpp
//
#include "ShapeConvex.h"
#include "Math/Helpers.h"
#include <functional>
#include <unordered_map>

/*
========================================================================================================

ShapeConvex

========================================================================================================
*/
template<typename T, typename Func>
int FindMax(const T* elems, size_t num, Func dist)
{
	int maxIdx = 0;
	float maxDist = dist(elems[0]);

	for (int i = 1;i < num; i++)
	{
		float currentDist = dist(elems[i]);
		if (currentDist > maxDist)
		{
			maxIdx = i;
		}
	}
	return maxIdx;
}


/*
================================
IsEdgeUnique
This will compare the incoming edge with all the edges in the facing tris
and then return true if it's unique
================================
*/
bool IsEdgeUnique(
	const std::vector<ConvexTriangle>& tris,
	const std::vector<int>& facingTris,
	const int ignoreTri,
	const ConvexEdge& edge
) {
	for (int i = 0; i < facingTris.size(); i++) {
		const int triIdx = facingTris[i];
		if (ignoreTri == triIdx) {
			continue;
		}

		const ConvexTriangle& tri = tris[triIdx];
		ConvexEdge edges[3];

		edges[0].a = tri.a;
		edges[0].b = tri.b;

		edges[1].a = tri.b;
		edges[1].b = tri.c;

		edges[2].a = tri.c;
		edges[2].b = tri.a;

		for (int e = 0; e < 3; e++) {
			if (edge == edges[e]) {
				return false;
			}
		}
	}
	return true;
}

/*
================================
AddPoint
================================
*/
void AddPoint(
	std::vector<Vec3>& hullPoints,
	std::vector<ConvexTriangle>& hullTris,
	const Vec3& pt
) {
	// This point is outside
	// Now we need to remove old triangles and build new ones
	// Find all the triangles that face this point
	std::vector<int> facingTris;
	for (int i = (int)hullTris.size() - 1; i >= 0; i--) {
		const ConvexTriangle& tri = hullTris[i];
		const Vec3& a = hullPoints[tri.a];
		const Vec3& b = hullPoints[tri.b];
		const Vec3& c = hullPoints[tri.c];
		const float dist = DistanceFromTriangle(a, b, c, pt);
		if (dist > 0.0f) {
			facingTris.push_back(i);
		}
	}

	// Now find all edges that are unique to the tris,
	// these will be the edges that form the new triangles
	std::vector<ConvexEdge> uniqueEdges;
	for (int i = 0; i < facingTris.size(); i++) {
		const int triIdx = facingTris[i];
		const ConvexTriangle& tri = hullTris[triIdx];
		ConvexEdge edges[3];

		edges[0].a = tri.a;
		edges[0].b = tri.b;
		edges[1].a = tri.b;
		edges[1].b = tri.c;
		edges[2].a = tri.c;
		edges[2].b = tri.a;

		for (int e = 0; e < 3; e++) {
			if (IsEdgeUnique(hullTris, facingTris, triIdx, edges[e])) {
				uniqueEdges.push_back(edges[e]);
			}
		}
	}

	// Now remove the old facing tris
	for (int i = 0; i < facingTris.size(); i++) {
		hullTris.erase(hullTris.begin() + facingTris[i]);
	}

	// Now add the new point
	hullPoints.push_back(pt);
	const int newPtIdx = (int)hullPoints.size() - 1;

	// Now add triangles for each unique edge
	for (int i = 0; i < uniqueEdges.size(); i++) {
		const ConvexEdge& edge = uniqueEdges[i];
		ConvexTriangle tri;
		tri.a = edge.a;
		tri.b = edge.b;
		tri.c = newPtIdx;
		hullTris.push_back(tri);
	}
}


/*
================================
RemoveUnreferencedVerts
================================
*/
void RemoveUnreferencedVerts(std::vector<Vec3>& hullPoints, std::vector<ConvexTriangle>& hullTris) {
	for (int i = 0; i < hullPoints.size(); i++) {
		bool isUsed = false;

		// Check if the point is used in any triangle
		for (int j = 0; j < hullTris.size(); j++) {
			const ConvexTriangle& tri = hullTris[j];
			if (tri.a == i || tri.b == i || tri.c == i) {
				isUsed = true;
				break;
			}
		}

		if (isUsed) {
			continue;
		}

		// Adjust triangle indices greater than i
		for (int j = 0; j < hullTris.size(); j++) {
			ConvexTriangle& tri = hullTris[j];
			if (tri.a > i) tri.a--;
			if (tri.b > i) tri.b--;
			if (tri.c > i) tri.c--;
		}

		// Remove the unused point
		hullPoints.erase(hullPoints.begin() + i);
		i--;  // Recheck the current index after removal
	}
}

/*
================================
RemoveInternalPoints
================================
*/
void RemoveInternalPoints(
	const std::vector<Vec3>& hullPoints,
	const std::vector<ConvexTriangle>& hullTris,
	std::vector<Vec3>& checkPts
) {
	// 移除位于当前凸包内部的点
	for (int i = 0; i < checkPts.size(); i++) {
		const Vec3& pt = checkPts[i];
		bool isExternal = false;

		for (int t = 0; t < hullTris.size(); t++) {
			const ConvexTriangle& tri = hullTris[t];
			const Vec3& a = hullPoints[tri.a];
			const Vec3& b = hullPoints[tri.b];
			const Vec3& c = hullPoints[tri.c];

			// 如果点位于任意一个三角面前方，则说明它在外部
			float dist = DistanceFromTriangle(a, b, c, pt);
			if (dist > 0.0f) {
				isExternal = true;
				break;
			}
		}

		// 若点不在任何面前方，则处于凸包内部，应移除
		if (!isExternal) {
			checkPts.erase(checkPts.begin() + i);
			i--;
		}
	}

	// 移除与凸包点距离过近的点（小于 1cm）
	for (int i = 0; i < checkPts.size(); i++) {
		const Vec3& pt = checkPts[i];
		bool isTooClose = false;

		for (int j = 0; j < hullPoints.size(); j++) {
			Vec3 hullPt = hullPoints[j];
			Vec3 ray = hullPt - pt;

			if (ray.GetLengthSqr() < 0.01f * 0.01f) {
				isTooClose = true;
				break;
			}
		}

		if (isTooClose) {
			checkPts.erase(checkPts.begin() + i);
			i--;
		}
	}
}

/*
================================
ExpandConvexHull
================================
*/
void ExpandConvexHull(
	std::vector<Vec3>& hullPoints,
	std::vector<ConvexTriangle>& hullTris,
	const std::vector<Vec3>& verts
) {
	// 按以下步骤扩展四面体，构造凸包
	// 1. 选择一个外部点
	// 2. 找到距离该点最远的点
	// 3. 将该点作为凸包新顶点
	// 4. 将新凸包内所有内部顶点移除，回到步骤1直到没有外部点

	std::vector<Vec3> externalVerts = verts;

	RemoveInternalPoints(hullPoints, hullTris, externalVerts);

	while (externalVerts.size() > 0) {
		Vec3 chosenVert = externalVerts[0];
		int ptIdx = FindMax(externalVerts.data(), externalVerts.size(),
				[&chosenVert](const Vec3& pt)
				{
					return chosenVert.Dot(pt);
				}
			);

		Vec3 pt = externalVerts[ptIdx];

		// Remove this element
		externalVerts.erase(externalVerts.begin() + ptIdx);

		AddPoint(hullPoints, hullTris, pt);
		RemoveInternalPoints(hullPoints, hullTris, externalVerts);
	}

	RemoveUnreferencedVerts(hullPoints, hullTris);
}




void FindConvexEdgesAndAdj(const std::vector<ConvexTriangle>& hullTris, std::vector<ConvexEdge>& hullEdges, std::vector<FaceAdjFaces>& hullAdjFaces)
{
	hullAdjFaces.resize(hullTris.size(), FaceAdjFaces{});
	std::unordered_map<ConvexEdge, int> edges;
	for (int faceIdx = 0; faceIdx < hullTris.size(); faceIdx++)
	{
		ConvexTriangle tri = hullTris[faceIdx];
		ConvexEdge triangleEdges[3] = { ConvexEdge(tri.a, tri.b) , ConvexEdge(tri.b, tri.c) , ConvexEdge(tri.c, tri.a) };
		for (int i = 0; i < 3; i++)
		{
			auto edgePos = edges.find(triangleEdges[i]);
			if (edgePos != edges.end())
			{
				FaceAdjFaces& face1 = hullAdjFaces[edgePos->second];
				FaceAdjFaces& face2 = hullAdjFaces[faceIdx];

				face1.faceIndex[face1.faceAdjCount] = faceIdx;
				face1.faceAdjCount++;

				face2.faceIndex[face2.faceAdjCount] = edgePos->second;
				face2.faceAdjCount++;
			}
			else
			{
				edges[triangleEdges[i]] = faceIdx;
			}
		}
	}

	for (auto& edge : edges)
	{
		hullEdges.push_back(edge.first);
	}
}

void BuildConvexHull(const std::vector< Vec3 >& verts, std::vector< Vec3 >& hullPts, std::vector< ConvexTriangle >& hullTris, std::vector<ConvexEdge>& hullEdges, std::vector<FaceAdjFaces>& hullAdjFaces)
{
	// 构建四面体作为初始条件
	// 1.找到最远两个点
	std::vector<int> excepts;

	int idx0 = FindMax(verts.data(), verts.size(),
		[](const Vec3& pt) -> float
		{
			return pt.Dot(Vec3(1, 0, 0));
		}
	);
	
	Vec3 pt0 = verts[idx0];
	int idx1 = FindMax(verts.data(), verts.size(),
		[&pt0](const Vec3& pt) -> float
		{
			return pt.Dot(pt0 * -1);
		}
	);
	Vec3 pt1 = verts[idx1];

	// 找到距离直线最远的点
	int idx2 = FindMax(verts.data(), verts.size(),
		[&pt0, &pt1](const Vec3& pt) -> float
		{
			return DistanceFromLine(pt0, pt1, pt);
		}
	);
	Vec3 pt2 = verts[idx2];

	Vec3 planeNormal = (pt1 - pt0).Cross(pt2 - pt0);
	// 找到距离三角形最远的点
	int idx3 = FindMax(verts.data(), verts.size(),
		[&pt0, &pt1, &pt2, &planeNormal](const Vec3& pt) -> float
		{
			//Vec3 normal = (pt1 - pt0).Cross(pt2 - pt0);
			return DistanceFromPlane(planeNormal, pt0, pt);
		}
	);
	Vec3 pt3 = verts[idx3];

	// 构建四面体
	if (DistanceFromPlane(planeNormal, pt0, pt3) > 0.0f)
	{
		std::swap(pt0, pt1);
	}

	hullPts.push_back(pt0);
	hullPts.push_back(pt1);
	hullPts.push_back(pt2);
	hullPts.push_back(pt3);
	hullTris.push_back(ConvexTriangle{ 0, 1, 2 });
	hullTris.push_back(ConvexTriangle{ 0, 2, 3 });
	hullTris.push_back(ConvexTriangle{ 2, 1, 3 });
	hullTris.push_back(ConvexTriangle{ 1, 0, 3 });

	ExpandConvexHull(hullPts, hullTris, verts);
	FindConvexEdgesAndAdj(hullTris, hullEdges, hullAdjFaces);
}

/*
====================================================
ShapeConvex::Build
====================================================
*/

void ShapeConvex::Build( const Vec3 * pts, const int num ) {
	// TODO: Add code
	std::vector<Vec3> verts(pts, pts + num);
	BuildConvexHull(verts, m_points, m_triangles, m_edges, m_adjFaces);

	// 计算凸包质心
	for (int i = 0;i < num; i++)
	{
		m_centerOfMass += pts[i] * (1.0f / num);
	}

	// 计算凸包转动惯量
	for (int vi = 0;vi < num; vi++)
	{
		Vec3 pt = m_points[vi] - m_centerOfMass;

		m_inertiaTensor.rows[0][0] +=  pt.y * pt.y + pt.z * pt.z;
		m_inertiaTensor.rows[1][0] += -pt.x * pt.y;
		m_inertiaTensor.rows[2][0] += -pt.x * pt.z;

		m_inertiaTensor.rows[0][1] += -pt.x * pt.y;
		m_inertiaTensor.rows[1][1] +=  pt.x * pt.x + pt.z * pt.z ;
		m_inertiaTensor.rows[2][1] += -pt.y * pt.z;

		m_inertiaTensor.rows[0][2] += -pt.x * pt.z;
		m_inertiaTensor.rows[1][2] += -pt.y * pt.z;
		m_inertiaTensor.rows[2][2] +=  pt.x * pt.x + pt.y * pt.y;
	}
	m_inertiaTensor *= (1.0f / num);

	// 构建完成后，计算凸包的各个顶角的位置
	m_coners[0] = m_bounds.mins;
	m_coners[1] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z);
	m_coners[2] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.mins.z);
	m_coners[3] = Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.mins.z);
	m_coners[4] = Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.maxs.z);
	m_coners[5] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.maxs.z);
	m_coners[6] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.maxs.z);
	m_coners[7] = m_bounds.maxs;
}

/*
====================================================
ShapeConvex::Support
====================================================
*/
Vec3 ShapeConvex::Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const {
	Vec3 supportPt;

	// 找到距离最远的顶点
	Vec3 maxPt = orient.RotatePoint(m_points[0]);
	float maxDist = dir.Dot(maxPt);
	for (int i = 1; i < m_points.size(); i++)
	{
		Vec3 pt = orient.RotatePoint(m_points[i]);
		float dist = dir.Dot(pt);
		if (dist > maxDist)
		{
			maxDist = dist;
			maxPt = pt;
		}
	}

	return maxPt + maxPt.Dir() * bias;
}

/*
====================================================
ShapeConvex::GetBounds
====================================================
*/
Bounds ShapeConvex::GetBounds( const Vec3 & pos, const Quat & orient ) const {
	Bounds bounds;

	for (int i = 1; i < 8; i++)
	{
		Vec3 pt = pos + orient.RotatePoint(m_coners[i]);
		bounds.Expand(pt);
	}

	return bounds;
}

/*
====================================================
ShapeConvex::FastestLinearSpeed
====================================================
*/
float ShapeConvex::FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const {
	float maxLinearSpeed = 0;

	for (int i = 0; i < m_points.size(); i++)
	{
		Vec3 vel = angularVelocity.Cross(m_points[i]);
		maxLinearSpeed = Max(vel.Dot(dir), maxLinearSpeed);
	}

	return maxLinearSpeed;
}

std::optional<PointArrayAccessor> ShapeConvex::GetPointData()
{
	return PointArrayAccessor(m_points);
}

Vec3 ShapeConvex::GetConvexVertex(int idx, Vec3 positionWS, Quat oriWS)
{
	assert(idx < m_points.size() && idx >= 0);

	return oriWS.RotatePoint(m_points[idx]) + positionWS;
}

std::vector<Vec3> ShapeConvex::FindClosestFaceByNormal(Vec3 normalWS, Vec3 positionWS, Quat oriWS, Vec3& normal, int& faceIdx)
{
	float closestDistance = -1.0f;
	std::vector<Vec3> closestTriangle;
	for (auto& tri : m_triangles)
	{
		Vec3 p0, p1, p2;
		GetTransformedTriangleVertices(tri, positionWS, oriWS, p0, p1, p2);
		Vec3 currentNormal = TriangleNormal(p0, p1, p2);

		float distance = currentNormal.Dot(normalWS);
		if (distance > closestDistance)
		{
			closestDistance = distance;
			closestTriangle.clear();
			closestTriangle.push_back(p0);
			closestTriangle.push_back(p1);
			closestTriangle.push_back(p2);
			normal = currentNormal;
		}
	}

	return closestTriangle;
}

std::vector<Vec3> ShapeConvex::FindClosestEdgeByContact(Vec3 contactWS, Vec3 positionWS, Quat oriWS)
{
	float closestDistance = std::numeric_limits<float>::max();
	std::vector<Vec3> closestEdge;
	for (auto& edge : m_edges)
	{
		Vec3 p0, p1;
		GetTransformedEdgeVertices(edge, positionWS, oriWS, p0, p1);
		Vec2 lambda = SignedVolumePt(p0, p1, contactWS);

		float distance = ((p0 * lambda.x + p1 * lambda.y) - contactWS).GetLengthSqr();

		if (distance < closestDistance)
		{
			closestDistance = distance;
			closestEdge.clear();
			closestEdge.push_back(p0);
			closestEdge.push_back(p1);
		}
	}

	return closestEdge;
}

void ShapeConvex::GetTransformedTriangleVertices(ConvexTriangle tri, Vec3 positionWS, Quat oriWS, Vec3& p0, Vec3& p1, Vec3& p2)
{
	p0 = positionWS + oriWS.RotatePoint(m_points[tri.a]);
	p1 = positionWS + oriWS.RotatePoint(m_points[tri.b]);
	p2 = positionWS + oriWS.RotatePoint(m_points[tri.c]);
}

void ShapeConvex::GetTransformedEdgeVertices(ConvexEdge edge, Vec3 positionWS, Quat oriWS, Vec3& p0, Vec3& p1)
{
	p0 = positionWS + oriWS.RotatePoint(m_points[edge.a]);
	p1 = positionWS + oriWS.RotatePoint(m_points[edge.b]);
}

ConvexEdge::ConvexEdge(int a, int b)
{
	if (a > b) std::swap(a, b);
	this->a = a;
	this->b = b;
}


const std::vector<Vec3>& ShapeConvex::GetVertices() const
{
	return m_points;
}

FaceAdjFaces ShapeConvex::FindAdjFaces(int faceIdx)
{
	return m_adjFaces[faceIdx];
}

void ShapeConvex::GetFaceInfo(int faceIdx, Vec3 positionWS, Quat oriWS, Vec3& normal, Vec3& origin)
{
	Vec3 p0, p1, p2;
	GetTransformedTriangleVertices(m_triangles[faceIdx], positionWS, oriWS, p0, p1, p2);
	normal = TriangleNormal(p0, p1, p2);
	origin = (p0 + p1 + p2) / 3.0f;
}
