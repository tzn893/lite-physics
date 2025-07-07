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

int ShapeBox::GetSeperateAxis(std::vector<Vec3>& outAxis)
{
	outAxis.push_back(Vec3(0, 0, 1));
	outAxis.push_back(Vec3(0, 1, 0));
	outAxis.push_back(Vec3(1, 0, 0));

	return 3;
}

Vec3 ShapeBox::GetConvexVertex(int idx, Vec3 positionWS, Quat oriWS)
{
	return positionWS + oriWS.RotatePoint(m_pts[idx]);
}

std::vector<Vec3> ShapeBox::FindClosestFaceByNormal(Vec3 normalWS, Vec3 positionWS, Quat oriWS, Vec3& normal, int& faceIdx)
{
	static Vec3 normals[6] =
	{
		Vec3( 1, 0, 0),
		Vec3(-1, 0, 0),
		Vec3( 0, 1, 0),
		Vec3( 0,-1, 0),
		Vec3( 0, 0, 1),
		Vec3( 0, 0,-1)
	};

	static int faces[6][4] =
	{
		{1, 3, 7, 5},
		{0, 4, 6, 2},
		{6, 7, 3, 2},
		{0, 1, 5, 4},
		{4, 5, 7, 6},
		{0, 2, 3, 1}
	};

	float closestDistance = -1.0f;
	int closestAxisIdx = 0;
	for (int i = 0;i < 6; i++)
	{
		Vec3 currentNormal = oriWS.RotatePoint(normals[i]);
		float distance = currentNormal.Dot(normalWS);
		if (distance > closestDistance)
		{
			faceIdx = i;
			normal = currentNormal;
			closestAxisIdx = i;
			closestDistance = distance;
		}
	}

	std::vector<Vec3> closestFace;
	for (int i = 0;i < 4;i++)
	{
		closestFace.push_back(GetConvexVertex(faces[closestAxisIdx][i], positionWS, oriWS));
	}

	return closestFace;
}

std::vector<Vec3> ShapeBox::FindClosestEdgeByContact(Vec3 contactWS, Vec3 positionWS, Quat oriWS)
{
	static int edges[12][2] =
	{
		{0, 1}, {1, 3}, {3, 2}, {2, 0}, {1, 5}, {3, 7},
		{2, 6}, {4, 0}, {4, 5}, {5, 7}, {7, 6}, {6, 4}
	};

	float closestDistance = std::numeric_limits<float>::max();
	std::vector<Vec3> closestEdge;
	for (int i = 0; i < 12; i++)
	{
		Vec3 p0 = GetConvexVertex(edges[i][0], positionWS, oriWS);
		Vec3 p1 = GetConvexVertex(edges[i][1], positionWS, oriWS);

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

FaceAdjFaces ShapeBox::FindAdjFaces(int faceIdx)
{
	static int adjFaces[6][4] =
	{
		{2,3,4,5},
		{2,3,4,5},
		{0,1,4,5},
		{0,1,4,5},
		{0,1,2,3},
		{0,1,2,3}
	};

	FaceAdjFaces adj;
	for (int i = 0; i < 4; i++)
	{
		adj.faceIndex[i] = adjFaces[faceIdx][i];
	}
	adj.faceAdjCount = 4;

	return adj;
}

void ShapeBox::GetFaceInfo(int faceIdx, Vec3 positionWS, Quat oriWS, Vec3& normal, Vec3& origin)
{
	static Vec3 normals[6] =
	{
		Vec3(1, 0, 0),
		Vec3(-1, 0, 0),
		Vec3(0, 1, 0),
		Vec3(0,-1, 0),
		Vec3(0, 0, 1),
		Vec3(0, 0,-1)
	};

	static int faces[6][4] =
	{
		{1, 3, 7, 5},
		{0, 4, 6, 2},
		{6, 7, 3, 2},
		{0, 1, 5, 4},
		{4, 5, 7, 6},
		{0, 2, 3, 1}
	};

	normal = oriWS.RotatePoint(normals[faceIdx]);
	origin = positionWS + oriWS.RotatePoint(m_pts[faces[faceIdx][0]]);
}
