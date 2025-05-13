#include "Helpers.h"

bool SphereRayIntersection(Vec3 sphereCenter, float radius, Vec3 rayOrigin, Vec3 rayDir, float& t0, float& t1)
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

bool AABBRayIntersection(Vec3 boundMin, Vec3 boundMax, Vec3 rayOrigin, Vec3 rayDir, float& t0, float& t1)
{
	Vec3 tMin = (boundMin - rayOrigin) / rayDir;
	Vec3 tMax = (boundMax - rayOrigin) / rayDir;
	Vec3 b1 = Min(tMin, tMax);
	Vec3 b2 = Max(tMin, tMax);
	
	t0 = Max(Max(b1.x, b1.y), b1.z);
	t1 = Min(Min(b2.x, b2.y), b2.z);

	return t0 <= t1;
};
