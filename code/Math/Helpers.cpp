#include "Helpers.h"

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


