#include "Helpers.h"
#include "Math/Matrix.h"

bool SphereRayIntersection(const Vec3& sphereCenter, float radius, const Vec3& rayOrigin, const Vec3& rayDir, float& t0, float& t1)
{
	Vec3 m = sphereCenter - rayOrigin;

	float a = rayDir.Dot(rayDir);
	float b = m.Dot(rayDir);
	float c = m.Dot(m) - radius * radius;

	float delta = b * b - a * c;
	if (delta < 0)
	{
		return false;
	}

	float sqrtDelta = Sqrt(delta);
	t0 = (b - sqrtDelta) / a;
	t1 = (b + sqrtDelta) / a;

	return t1 >= 0;
}

bool AABBRayIntersection(const Vec3& boundMin, const Vec3& boundMax, const Vec3& rayOrigin, const Vec3& rayDir, float& t0, float& t1)
{
	Vec3 tMin = (boundMin - rayOrigin) / rayDir;
	Vec3 tMax = (boundMax - rayOrigin) / rayDir;
	Vec3 b1 = Min(tMin, tMax);
	Vec3 b2 = Max(tMin, tMax);
	
	t0 = Max(Max(b1.x, b1.y), b1.z);
	t1 = Min(Min(b2.x, b2.y), b2.z);

	return t0 <= t1;
}

float DistanceFromLine(const Vec3& a, const Vec3& b, const Vec3& pt)
{
	Vec3 ab = b - a;
	ab.Normalize();

	Vec3 ray = pt - a;
	Vec3 projection = ab * ray.Dot(ab);
	Vec3 perpindicular = ray - projection;

	return perpindicular.GetMagnitude();
}

float DistanceFromPlane(const Vec3& normal, const Vec3& planePt, const Vec3& pt)
{
	Vec3 ray = pt - planePt;
	return ray.Dot(normal);
}

float DistanceFromTriangle(const Vec3& pt0, const Vec3& pt1, const Vec3& pt2, const Vec3& pt)
{
	Vec3 planeNormal = TriangleNormal(pt0, pt1, pt2);

	return DistanceFromPlane(planeNormal, pt0, pt);
}

Vec3 TriangleNormal(const Vec3& pt0, const Vec3& pt1, const Vec3& pt2)
{
	return ((pt1 - pt0).Cross(pt2 - pt0)).Dir();
}



// Signed Volume算法，将点pt投影到pt0以及pt1构成的线段上
Vec2 SignedVolume(Vec3 pt0, Vec3 pt1)
{
	Vec3 pt0pt1 = pt1 - pt0;
	Vec3 pt0pt = Vec3(0.0) - pt0;

	// 投影到线段上的点pt
	Vec3 projPt = pt0 + pt0pt1 * (pt0pt1.Dot(pt0pt)) / pt0pt1.GetLengthSqr();

	int maxAxis = 0;
	float muMax = 0;
	for (int i = 0; i < 3; i++)
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
	else if ((a < b && c < a) || (a > b && c > a))
	{
		return Vec2(1.0f, 0.0f);
	}

	return Vec2(0.0f, 1.0f);
}

// Signed Volume算法，将点pt投影到pt0,pt1以及pt2构成的三角形上
Vec3 SignedVolume(Vec3 pt0, Vec3 pt1, Vec3 pt2)
{
	Vec3 normal = (pt1 - pt0).Cross(pt2 - pt0);
	normal.Normalize();
	Vec3 projPt = normal * normal.Dot(pt0);

	// 找到投影面积最大的面
	int maxAxis = 0;
	float areaMax = 0;
	for (int i = 0; i < 3; i++)
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

	// 将点投影到对应轴上
	int j = (maxAxis + 1) % 3;
	int k = (maxAxis + 2) % 3;
	Vec2 s[3];
	s[0] = Vec2(pt0[j], pt0[k]);
	s[1] = Vec2(pt1[j], pt1[k]);
	s[2] = Vec2(pt2[j], pt2[k]);
	Vec2 p = Vec2(projPt[j], projPt[k]);

	// 并计算三个子三角形的面积
	Vec3 areas;
	for (int i = 0; i < 3; i++)
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

	// 如果投影点在三角形内部，则返回重心坐标
	int sign = Sign(areaMax);
	if (sign == Sign(areas[0]) && sign == Sign(areas[1]) && sign == Sign(areas[2]))
	{
		return areas / areaMax;
	}

	// 若投影点在三角形外，则将点投影到三条边上，找到最近点
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

// Signed Volume算法，检查点pt距离三棱锥最近的点
Vec4 SignedVolume(Vec3 pt0, Vec3 pt1, Vec3 pt2, Vec3 pt3)
{
	// 计算四个子四棱柱的体积
	Mat4 mat;
	mat.rows[0] = Vec4(pt0[0], pt1[0], pt2[0], pt3[0]);
	mat.rows[1] = Vec4(pt0[1], pt1[1], pt2[1], pt3[1]);
	mat.rows[2] = Vec4(pt0[2], pt1[2], pt2[2], pt3[2]);
	mat.rows[3] = Vec4(1, 1, 1, 1);

	Vec4 volumes;
	volumes[0] = mat.Cofactor(3, 0);
	volumes[1] = mat.Cofactor(3, 1);
	volumes[2] = mat.Cofactor(3, 2);
	volumes[3] = mat.Cofactor(3, 3);

	// 计算总体积
	float totalVolume = volumes[0] + volumes[1] + volumes[2] + volumes[3];
	int totalVolumeSign = Sign(totalVolume);

	// 若点在内部则返回内部坐标
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

	// 否则计算各个点在面元上的投影
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