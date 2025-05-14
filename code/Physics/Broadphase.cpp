//
//  Broadphase.cpp
//
#include "Broadphase.h"
#include <algorithm>

/*
====================================================
BroadPhase
====================================================
*/
struct BodyPoint
{
	int id;
	bool isMin;
	float dist;
};

void SweepAndSort(std::vector<BodyPoint>& bodyPt, const std::vector<Body*>& bodies, float dt)
{
	Vec3 axis = Vec3(1, 1, 1);

	for (int i = 0;i < bodies.size();i++)
	{
		Body* body = bodies[i];
		Bounds bounds = body->GetBounds();

		// 适当扩大采用了CCD的body以保证CCD能有效执行
		bounds.Expand(bounds.mins - body->GetLinearVelocity() * dt - Vec3(1, 1, 1) * 1e-4);
		bounds.Expand(bounds.maxs + body->GetLinearVelocity() * dt + Vec3(1, 1, 1) * 1e-4);

		float dist0 = bounds.mins.Dot(axis);
		float dist1 = bounds.maxs.Dot(axis);

		bodyPt.push_back({ i, true, dist0 });
		bodyPt.push_back({ i, false, dist1 });
	}

	std::sort
	(
		bodyPt.begin(),
		bodyPt.end(),
		[](const BodyPoint& a, const BodyPoint& b)
		{
			return a.dist < b.dist;
		}
	);
}

void BuildPairs(const std::vector<BodyPoint>& bodyPt, std::vector< collisionPair_t >& finalPairs)
{
	for (int i = 0;i < bodyPt.size();i++)
	{
		if (!bodyPt[i].isMin) continue;
		int leftId = bodyPt[i].id;

		for (int j = i + 1; j < bodyPt.size(); j++)
		{
			if (bodyPt[j].id == leftId) break;
			
			if (!bodyPt[j].isMin) continue;

			collisionPair_t pair;
			pair.a = leftId;
			pair.b = bodyPt[j].id;

			finalPairs.push_back(pair);
		}
	}
}



void BroadPhase(const std::vector<Body*>& bodies, std::vector< collisionPair_t > & finalPairs, const float dt_sec )
{
	// TODO: Add Code
	std::vector<BodyPoint> bodyPt;

	SweepAndSort(bodyPt, bodies, dt_sec);
	BuildPairs(bodyPt, finalPairs);
}