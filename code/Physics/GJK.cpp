//
//  GJK.cpp
//
#include "GJK.h"
#include "Math/Helpers.h"
#include <tuple>

template<typename T>
using opt = std::optional <T>;

template<typename ...Args>
using tpl = std::tuple<Args...>;

// Signed Volume�㷨������ptͶӰ��pt0�Լ�pt1���ɵ��߶���
Vec2 SignedVolume(Vec3 pt0, Vec3 pt1)
{
	Vec3 pt0pt1 = pt1 - pt0;
	Vec3 pt0pt = Vec3(0.0) - pt0;

	// ͶӰ���߶��ϵĵ�pt
	Vec3 projPt = pt0 + pt0pt1 * (pt0pt1.Dot(pt0pt)) / pt0pt1.GetLengthSqr();

	int maxAxis = 0;
	float muMax = 0;
	for (int i = 0;i < 3; i++)
	{
		int mu = pt1[i] - pt0[i];
		if (mu * mu > muMax * muMax)
		{
			muMax = mu;
			maxAxis = i;
		}
	}

	float a = pt0[maxAxis];
	float b = pt1[maxAxis];
	float c = projPt[maxAxis];

	if ((a < c && c < b) || (a > c && c > b))
	{
		Vec2 rv = Vec2(b - c, c - a);
		return rv / (b - a);
	}
	else if((a < b && c < a) || (a > b && c > a))
	{
		return Vec2(1.0f, 0.0f);
	}

	return Vec2(0.0f, 1.0f);
}

// Signed Volume�㷨������ptͶӰ��pt0,pt1�Լ�pt2���ɵ���������
Vec3 SignedVolume(Vec3 pt0, Vec3 pt1, Vec3 pt2)
{
	Vec3 normal = (pt1 - pt0).Cross(pt2 - pt0);
	normal.Normalize();
	Vec3 projPt = normal * normal.Dot(pt0);

	// �ҵ�ͶӰ���������
	int maxAxis = 0;
	float areaMax = 0;
	for (int i = 0;i < 3;i++)
	{
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;
		Vec2 a = Vec2(pt0[j], pt0[k]);
		Vec2 b = Vec2(pt1[j], pt1[k]);
		Vec2 c = Vec2(pt2[j], pt2[k]);

		Vec2 ab = b - a;
		Vec2 ac = c - a;

		float area = ab.x * ac.y - ab.y * ac.x;
		if (area * area > areaMax * areaMax)
		{
			maxAxis = i;
			areaMax = area;
		}

	}

	// ����ͶӰ����Ӧ����
	int j = (maxAxis + 1) % 3;
	int k = (maxAxis + 2) % 3;
	Vec2 s[3];
	s[0] = Vec2(pt0[j], pt0[k]);
	s[1] = Vec2(pt1[j], pt1[k]);
	s[2] = Vec2(pt2[j], pt2[k]);
	Vec2 p = Vec2(projPt[j], projPt[k]);

	// �����������������ε����
	Vec3 areas;
	for (int i = 0;i < 3; i++)
	{
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;

		Vec2 a = p;
		Vec2 b = s[j];
		Vec2 c = s[k];
		Vec2 ab = b - a;
		Vec2 ac = c - a;

		areas[i] = ab.x * ac.y - ab.y * ac.x;
	}

	// ���ͶӰ�����������ڲ����򷵻���������
	int sign = Sign(areaMax);
	if (sign == Sign(areas[0]) && sign == Sign(areas[1]) && sign == Sign(areas[2]))
	{
		return areas / areaMax;
	}

	// ��ͶӰ�����������⣬�򽫵�ͶӰ���������ϣ��ҵ������
	float dist = 1e10f;
	Vec3 rv = Vec3(1, 0, 0);
	
	Vec3 edgePts[3];
	edgePts[0] = pt0;
	edgePts[1] = pt1;
	edgePts[2] = pt2;
	
	for (int i = 0; i < 3; i++)
	{
		int k = (i + 1) % 3;
		int j = (i + 2) % 3;

		Vec2 lambdaEdge = SignedVolume(edgePts[k], edgePts[j]);
		Vec3 pt = edgePts[k] * lambdaEdge[0] + edgePts[j] * lambdaEdge[1];
		if (pt.GetLengthSqr() < dist)
		{
			dist = pt.GetLengthSqr();
			rv[i] = 0;
			rv[k] = lambdaEdge[0];
			rv[j] = lambdaEdge[1];
		}
	}

	return rv;
}



// Signed Volume�㷨������pt��������׶����ĵ�
Vec4 SignedVolume(Vec3 pt0, Vec3 pt1, Vec3 pt2, Vec3 pt3)
{
	// �����ĸ��������������
	Mat4 mat;
	mat.rows[0] = Vec4(pt0[0], pt1[0], pt2[0], pt3[0]);
	mat.rows[1] = Vec4(pt0[1], pt1[1], pt2[1], pt3[1]);
	mat.rows[2] = Vec4(pt0[2], pt1[2], pt2[2], pt3[2]);
	mat.rows[3] = Vec4(		1,		1,		1,		1);

	Vec4 volumes;
	volumes[0] = mat.Cofactor(3, 0);
	volumes[1] = mat.Cofactor(3, 1);
	volumes[2] = mat.Cofactor(3, 2);
	volumes[3] = mat.Cofactor(3, 3);

	// ���������
	float totalVolume = volumes[0] + volumes[1] + volumes[2] + volumes[3];
	int totalVolumeSign = Sign(totalVolume);

	// �������ڲ��򷵻��ڲ�����
	if (totalVolumeSign == Sign(volumes[0]) && totalVolumeSign == Sign(volumes[1])
		&& totalVolumeSign == Sign(volumes[2]) && totalVolumeSign == Sign(volumes[3]))
	{
		return volumes / totalVolume;
	}

	Vec3 facePts[4];
	facePts[0] = pt0;
	facePts[1] = pt1;
	facePts[2] = pt2;
	facePts[3] = pt3;

	// ����������������Ԫ�ϵ�ͶӰ
	Vec4 lambdas;
	float dist = 1e10f;
	for (int i = 0; i < 4; i++)
	{
		int j = (i + 1) % 4;
		int k = (i + 2) % 4;
		int l = (i + 3) % 4;

		Vec3 lambdaThe = SignedVolume(facePts[i], facePts[j], facePts[k]);
		Vec3 p = facePts[i] * lambdaThe[0] + facePts[j] * lambdaThe[1] 
			+ facePts[k] * lambdaThe[2];

		if (p.GetLengthSqr() < dist)
		{
			dist = p.GetLengthSqr();
			lambdas[i] = lambdaThe[0];
			lambdas[j] = lambdaThe[1];
			lambdas[k] = lambdaThe[2];
			lambdas[l] = 0;
		}
	}

	return lambdas;
}
/*
================================
GJK_DoesIntersect
================================
*/
void EPASolver::Solve(const Body* bodyA, const Body* bodyB, float bias, MkDifferencePoint* simplexPts
	, Vec3& ptOnA, Vec3& ptOnB)
{
	triangles.clear();
	points = std::vector<MkDifferencePoint>(simplexPts, simplexPts + 4);

	// ��������������ĵ�
	Vec3 center = Vec3();
	for (int i = 0; i < 4; i++)
	{
		center += points[i].pt;
	}
	center *= 0.25f;


	// �����������Ӧ������
	for (int i = 0; i < 4; i++)
	{
		int j = (i + 1) % 4;
		int k = (i + 2) % 4;
		int l = (i + 3) % 4;

		ConvexTriangles tri{ i, j, k };

		// �����η���Ӧ�����⣬����ζ����һ���㵽�����εľ������С��0��������þ������0����Ҫ���������ζ���˳��
		if (DistanceFromTriangle(points[tri.a].pt, points[tri.b].pt, points[tri.c].pt,
			points[l].pt) > 0)
		{
			std::swap(tri.b, tri.c);
		}

		triangles.push_back(tri);
	}


	while (true)
	{
		// �ҵ���ǰ����ԭ������������Σ������䷨�߷�������
		int closestTriangleIdx = FindClosestTriangle();
		ConvexTriangles closestTriangle = triangles[closestTriangleIdx];

		Vec3 closestTriangleNormal = TriangleNormal(points[closestTriangle.a].pt,
			points[closestTriangle.b].pt, points[closestTriangle.c].pt);

		MkDifferencePoint newPt = MkDifferencePoint::Support(bodyA, bodyB, closestTriangleNormal, bias);


		// ����µ�����͹���ڲ���˵���޷���һ�����ţ�������ײ�����
		if (HasPoint(newPt) || SignedDistanceToTriangle(closestTriangle, newPt.pt) <= 1e-4f)
		{
			// �ҵ�ԭ��ͶӰ��������ϵ���������
			MkDifferencePoint closestTrianglePts[3] = { points[closestTriangle.a],
				points[closestTriangle.b], points[closestTriangle.c] };

			Vec3 lambda = SignedVolume(closestTrianglePts[0].pt,
				closestTrianglePts[1].pt, closestTrianglePts[2].pt);

			MkDifferencePoint closestPt = MkDifferencePoint::SimplexInterpolate(closestTrianglePts, 3,
				Vec4(lambda.x, lambda.y, lambda.z, 0));

			ptOnA = closestPt.ptOnA;
			ptOnB = closestPt.ptOnB;

			return;
		}

		// ���µ����������Ķ�����
		points.push_back(newPt);
		
		/*
		// ���������δ�͹�����Ƴ��������µ�������
		triangles.erase(triangles.begin() + closestTriangleIdx);

		int newPointIdx = points.size() - 1;
		int triangleIdx[3] = { closestTriangle.a, closestTriangle.b, closestTriangle.c };
		for (int i = 0; i < 3; i++)
		{
			int j = (i + 1) % 3;

			ConvexTriangles tri{ newPointIdx, triangleIdx[i], triangleIdx[j] };

			if (DistanceFromTriangle(points[tri.a].pt, points[tri.b].pt, points[tri.c].pt, Vec3(0, 0, 0)) > 0)
			{
				std::swap(tri.b, tri.c);
			}

			triangles.push_back(tri);
		}
		*/

		// ��͹�����Ƴ������µ��������
		RemovePointFacingTriangle(newPt.pt);
		// ���µ������������
		FillTrianglesWithNewPoint();
	}

}


// �ҵ�����ԭ�������������
int EPASolver::FindClosestTriangle()
{
	int minIdx = 0;
	float minDistance = 1e10;

	for (int idx = 0; idx < triangles.size(); idx++)
	{
		ConvexTriangles tri = triangles[idx];

		// ���������η��߷�����ԭ���෴����˾���һ��С��0
		float distance = abs(SignedDistanceToTriangle(tri));
		// assert(distance >= 0.0f);
		if (distance < minDistance)
		{
			minDistance = distance;
			minIdx = idx;
		}
	}

	return minIdx;
}

float EPASolver::SignedDistanceToTriangle(const ConvexTriangles& tri, const Vec3& pt)
{
	return DistanceFromTriangle(points[tri.a].pt, points[tri.b].pt, points[tri.c].pt, pt);
}

	// ���һ���µ��Ƿ��Ѿ��ڱ������͹���ڲ�
bool EPASolver::HasPoint(const MkDifferencePoint& pt)
{
	for (int idx = 0; idx < points.size(); idx++)
	{
		if ((pt.pt - points[idx].pt).GetLengthSqr() < 1e-8)
		{
			return true;
		}
	}
	return false;
}

void EPASolver::RemovePointFacingTriangle(const Vec3& pt)
{
	for (int idx = 0; idx < triangles.size();)
	{
		ConvexTriangles tri = triangles[idx];
		// �����β�Ӧ�������¼���ĵ�
		if (SignedDistanceToTriangle(tri, pt) >= 0)
		{
			triangles.erase(triangles.begin() + idx);
		}
		else
		{
			idx++;
		}
	}
}



void EPASolver::FillTrianglesWithNewPoint()
{
	// �ҵ��������е���ͨ�ı�
	struct Edge
	{
		int a = 0, b = 0;
		Edge() = default;
		Edge(int a, int b)
		{
			this->a = Min(a, b);
			this->b = Max(a, b);
		}

		bool operator==(const Edge& e)
		{
			return a == e.a && b == e.b;
		}
	};
	
	// ������ײ�������������ܱȽ�С���ø������ݽṹ���ٷ������ܸ��Ż�
	// �������������εıߣ�����߲��ڶ����оͼ��뵽�����У����򽫱ߴӶ������Ƴ�
	// ���ڷǹ����߻ᱻ�����湲�ã���˱�������ʣ��ı�Ϊ������
	std::vector<Edge> danglingEdges;
	for (int i = 0;i < triangles.size(); i++)
	{
		int triangleIdx[3] = { triangles[i].a, triangles[i].b, triangles[i].c };
		for (int j = 0; j < 3; j++)
		{
			int k = (j + 1) % 3;
			if (auto edgePos = std::find(danglingEdges.begin(), danglingEdges.end(), 
					Edge(triangleIdx[j], triangleIdx[k])); edgePos != danglingEdges.end())
			{
				danglingEdges.erase(edgePos);
			}
			else
			{
				danglingEdges.push_back(Edge(triangleIdx[j], triangleIdx[k]));
			}
		}
	}

	// �ӹ������Լ����һ�����㹹��������
	for (auto& edge : danglingEdges)
	{
		int lastPtIdx = points.size() - 1;
		ConvexTriangles tri{ lastPtIdx, edge.a, edge.b };
		// ��֤�����εķ������ǳ���
		if (SignedDistanceToTriangle(tri) > 0)
		{
			std::swap(tri.b, tri.c);
		}
		triangles.push_back(tri);
	}
}




// ֧��1, 2, 3����simplex��Signed Volume����
// ��ԭ����simplex�ڲ�ʱ������{�Ƿ����ԭ�㣬�������Ȩ�أ�ͶӰ����λ��}
tpl<bool, Vec4, Vec3> GeneralSignedVolume(int simplexCnt,const MkDifferencePoint* pts)
{
	assert(simplexCnt >= 2 && simplexCnt <= 4);

	Vec4 lambda;
	bool inSide = false;

	switch (simplexCnt)
	{
	case 2:
		{
			Vec2 lambda2D = SignedVolume(pts[0].pt, pts[1].pt);
			lambda.x = lambda2D.x;
			lambda.y = lambda2D.y;
		}
		break;
	case 3:
		{
			Vec3 lambda3D = SignedVolume(pts[0].pt, pts[1].pt, pts[2].pt);
			lambda.x = lambda3D.x;
			lambda.y = lambda3D.y;
			lambda.z = lambda3D.z;
		}
		break;
	case 4:
		lambda = SignedVolume(pts[0].pt, pts[1].pt, pts[2].pt, pts[3].pt);
		break;
	}

	// ԭ�㵽Simplex�������λ��
	Vec3 projectPt;
	for (int i = 0;i < simplexCnt;i++)
	{
		projectPt += pts[i].pt * lambda[i];
	}
	// ��ͶӰ�����ԭ���㹻����˵����body�ཻ
	inSide = projectPt.GetLengthSqr() < 1e-8f;

	return std::make_tuple(inSide, lambda, projectPt);
}

bool GJK_DoesIntersect( const Body * bodyA, const Body * bodyB ) 
{
	// TODO: Add code
	MkDifferencePoint supportPts[4];

	int simplexCnt = 1;
	supportPts[0] = MkDifferencePoint::Support(bodyA, bodyB, Vec3(1, 1, 1), 0.0f);

	Vec3 searchDir = Vec3(-1, -1, -1).Dir();

	float closestPointFromOriginSqr = supportPts[0].pt.GetLengthSqr();
	MkDifferencePoint closestPoint = supportPts[0];

	bool doseContainOrigin = false;
	bool noProgress = false;

	while (!doseContainOrigin && !noProgress)
	{
		MkDifferencePoint newSupportPt = MkDifferencePoint::Support(bodyA, bodyB, searchDir, 0.0f);

		// ����µ��Ƿ�����ʷ����ͬ������ͬ��֤���㷨�޷�ȡ�ø����չ�������岻�ཻ
		for (int i = 0; i < simplexCnt; i++)
		{
			if ((supportPts[i].pt - newSupportPt.pt).GetLengthSqr() < 1e-8f)
			{
				noProgress = true;
				break;
			}
		}
		if (noProgress) break;

		// ���¼���ĵ���������У�������͹��
		supportPts[simplexCnt++] = newSupportPt;

		// ������͹����Signed Volume
		auto [hasIntersection, lambda, projPt] = GeneralSignedVolume(simplexCnt, supportPts);
		doseContainOrigin = hasIntersection;

		// ����µ��޷����֮ǰ�޷�ȡ�ý�չ������ѭ�������㷨
		if (projPt.GetLengthSqr() >= closestPointFromOriginSqr)
		{
			break;
		}
		closestPointFromOriginSqr = projPt.GetLengthSqr();
		closestPoint = MkDifferencePoint::SimplexInterpolate(supportPts, simplexCnt, lambda);

		// ������һ��ѭ�����µķ���
		searchDir = (projPt * -1).Dir();

		// ��͹���ڵ㰴��Ȩ���Ƿ���Ч����
		int cIdx = 0, pIdx = 0;
		for (; cIdx < simplexCnt; cIdx++)
		{
			if (lambda[cIdx] != 0)
			{
				std::swap(supportPts[cIdx], supportPts[pIdx++]);
			}
		}
		// �µ�͹��������Ϊ��Ч������
		simplexCnt = pIdx;
	}

	return doseContainOrigin;
}

/*
================================
GJK_ClosestPoints
================================
*/
void GJK_ClosestPoints( const Body * bodyA, const Body * bodyB, Vec3 & ptOnA, Vec3 & ptOnB )
{
	GJK_DoesIntersect(bodyA, bodyB, 2e-4f, ptOnA, ptOnB);
}

/*
================================
GJK_DoesIntersect
================================
*/
bool GJK_DoesIntersect( const Body * bodyA, const Body * bodyB, const float bias, Vec3 & ptOnA, Vec3 & ptOnB ) 
{
	// TODO: Add code
	MkDifferencePoint supportPts[4];

	int simplexCnt = 1;
	supportPts[0] = MkDifferencePoint::Support(bodyA, bodyB, Vec3(1, 1, 1), bias);

	Vec3 searchDir = Vec3(-1, -1, -1).Dir();

	float closestPointFromOriginSqr = supportPts[0].pt.GetLengthSqr();
	MkDifferencePoint closestPoint = supportPts[0];

	bool doseContainOrigin = false;
	bool noProgress = false;

	while (!doseContainOrigin && !noProgress)
	{
		MkDifferencePoint newSupportPt = MkDifferencePoint::Support(bodyA, bodyB, searchDir, bias);

		// ����µ��Ƿ�����ʷ����ͬ������ͬ��֤���㷨�޷�ȡ�ø����չ�������岻�ཻ
		for (int i = 0; i < simplexCnt; i++)
		{
			if ((supportPts[i].pt - newSupportPt.pt).GetLengthSqr() < 1e-8f)
			{
				noProgress = true;
				break;
			}
		}

		// ����µ��޷����ԭ�㣬˵�������ܴ��ڽ���
		if (searchDir.Dot(newSupportPt.pt) < 0.0f)
		{
			noProgress = true;
		}

		if (noProgress) break;

		// ���¼���ĵ���������У�������͹��
		supportPts[simplexCnt++] = newSupportPt;

		// ������͹����Signed Volume
		auto [hasIntersection, lambda, projPt] = GeneralSignedVolume(simplexCnt, supportPts);
		doseContainOrigin = hasIntersection;

		float projPointFromOriginSqr = projPt.GetLengthSqr();
		// �µ�û�и��������ֱ���˳�ѭ��
		if (projPointFromOriginSqr >= closestPointFromOriginSqr)
		{
			noProgress = true;
			break;
		}
		closestPointFromOriginSqr = projPt.GetLengthSqr();
		closestPoint = MkDifferencePoint::SimplexInterpolate(supportPts, simplexCnt, lambda);
		
		// ������һ��ѭ�����µķ���
		searchDir = (projPt * -1).Dir();

		// ��͹���ڵ㰴��Ȩ���Ƿ���Ч����
		int cIdx = 0, pIdx = 0;
		for (; cIdx < simplexCnt; cIdx++)
		{
			if (lambda[cIdx] != 0)
			{
				std::swap(supportPts[cIdx], supportPts[pIdx++]);
			}
		}
		// �µ�͹��������Ϊ��Ч������
		simplexCnt = pIdx;
	}

	
	// ���������ཻʱ��GJK�õ��������һ���Ǿ�������ĵ㣬��Ҫʹ��EPA�㷨����
	// ��Ҫ����EPA���㽻��
	if (doseContainOrigin)
	{
		// �����˻��������simplex����Ϊ������
		if (simplexCnt == 1)
		{
			Vec3 Dir = supportPts[0].pt * -1;
			MkDifferencePoint pt = MkDifferencePoint::Support(bodyA, bodyB, Dir, bias);
			supportPts[simplexCnt++] = pt;
		}
		if (simplexCnt == 2)
		{
			Vec3 u, v;
			(supportPts[1].pt - supportPts[0].pt).GetOrtho(u, v);
			supportPts[simplexCnt++] = MkDifferencePoint::Support(bodyA, bodyB, u, bias);
		}
		if (simplexCnt == 3)
		{
			Vec3 normal = TriangleNormal(supportPts[0].pt, supportPts[1].pt, supportPts[2].pt);
			if (normal.Dot(supportPts[0].pt) < 0.0f)
			{
				std::swap(supportPts[0], supportPts[1]);
			}

			MkDifferencePoint pt = MkDifferencePoint::Support(bodyA, bodyB, normal, bias);
			supportPts[simplexCnt++] = pt;
		}

		EPASolver().Solve(bodyA, bodyB, bias, supportPts, ptOnA, ptOnB);
	}
	else
	{
		// ���������Ϊ��ײ���Ľ��
		ptOnA = closestPoint.ptOnA;
		ptOnB = closestPoint.ptOnB;
	}

	return doseContainOrigin;
}