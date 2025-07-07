//
//	Intersections.h
//
#pragma once
#include "Contact.h"

struct IntersectionManifold
{
	static constexpr int maxContactCount = 8;
	int contactCount;

	contact_t contacts[maxContactCount];

	IntersectionManifold();
	IntersectionManifold(const IntersectionManifold& rhs);

	bool HasIntersection();
};


bool Intersect( Body * bodyA, Body * bodyB, IntersectionManifold& manifold );
bool Intersect( Body * bodyA, Body * bodyB, const float dt, IntersectionManifold& manifold);