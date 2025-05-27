//
//	GJK.h
//
#pragma once
#include "../Math/Vector.h"
#include "../Math/Quat.h"
#include "../Math/Matrix.h"
#include "../Math/Bounds.h"
#include "Body.h"
#include "Shapes.h"

// �ڼ���͹��mikovski differenceʱ��͹���ϵ�ĳһ���Լ����Ӧ������͹���ϵĵ�
struct MkDifferencePoint
{
	Vec3 pt;
	Vec3 ptOnA;
	Vec3 ptOnB;

	MkDifferencePoint() = default;
	MkDifferencePoint(const MkDifferencePoint& other) = default;
	MkDifferencePoint(const Vec3& pt, const Vec3& ptOnA, const Vec3& ptOnB)
		: ptOnA(ptOnA), ptOnB(ptOnB), pt(pt)
	{
	}

	static MkDifferencePoint Support(const Body* bodyA, const Body* bodyB, const Vec3 Dir, float bias)
	{
		Vec3 ptOnA = bodyA->GetSupportWorldSpace(Dir, bias);
		Vec3 ptOnB = bodyB->GetSupportWorldSpace(Dir * -1, bias);
		return MkDifferencePoint(ptOnA - ptOnB, ptOnA, ptOnB);
	}

	// ��simplex�ĸ��������в�ֵ������õ�
	static MkDifferencePoint SimplexInterpolate(MkDifferencePoint* pts, int simplexCnt, Vec4 lambda)
	{
		MkDifferencePoint pt;
		for (int i = 0; i < simplexCnt; i++)
		{
			pt.pt += pts[i].pt * lambda[i];
			pt.ptOnA += pts[i].ptOnA * lambda[i];
			pt.ptOnB += pts[i].ptOnB * lambda[i];
		}
		return pt;
	}
};


struct ConvexTriangles
{
	int a, b, c;
};


class EPASolver
{
public:

	void Solve(const Body* bodyA,const Body* bodyB, float bias, MkDifferencePoint* simplexPts
		, Vec3& ptOnA, Vec3& ptOnB);


private:

	// �ҵ�����ԭ�������������
	int FindClosestTriangle();

	float SignedDistanceToTriangle(const ConvexTriangles& tri, const Vec3& pt = Vec3(0, 0, 0));

	// ���һ���µ��Ƿ��Ѿ��ڱ������͹���ڲ�
	bool HasPoint(const MkDifferencePoint& pt);

	// �������µ�������������޳���
	void RemovePointFacingTriangle(const Vec3& pt);

	// ��ȱ�������εı����������β���
	void FillTrianglesWithNewPoint();

	std::vector<MkDifferencePoint> points;
	std::vector<ConvexTriangles> triangles;
	Body* bodyA;
	Body* bodyB;
	float bias;
};


bool GJK_DoesIntersect( const Body * bodyA, const Body * bodyB );
bool GJK_DoesIntersect( const Body * bodyA, const Body * bodyB, const float bias, Vec3 & ptOnA, Vec3 & ptOnB);
void GJK_ClosestPoints( const Body * bodyA, const Body * bodyB, Vec3 & ptOnA, Vec3 & ptOnB);

Vec2 SignedVolume(Vec3 pt0, Vec3 pt1);
Vec3 SignedVolume(Vec3 pt0, Vec3 pt1, Vec3 pt2);
Vec4 SignedVolume(Vec3 pt0, Vec3 pt1, Vec3 pt2, Vec3 pt3);