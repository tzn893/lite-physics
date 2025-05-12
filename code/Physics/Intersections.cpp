//
//  Intersections.cpp
//
#include "Intersections.h"
#include "GJK.h"

#include <array>

#define p_count_of(arr) (sizeof(arr) / sizeof(arr[0]))

void PlanePlaneIntersect(Body* planeA, Body* planeB, contact_t& contact)
{
	// TODO
	assert(false);
}

void SpherePlaneIntersect(Body* sphereA, Body* planeB, contact_t& contact)
{
	assert(sphereA->GetShape()->GetType() == Shape::SHAPE_SPHERE &&
		planeB->GetShape()->GetType() == Shape::SHAPE_PLANE);

	ShapeSphere* shapeA = static_cast<ShapeSphere*>(sphereA->GetShape());
	ShapePlane* planeB = static_cast<ShapePlane*>(planeB->GetShape());


}

void PlaneShapeIntersect(Body* planeB, Body* sphereA, contact_t& contact)
{
	SpherePlaneIntersect(sphereA, planeB, contact);
}

void SphereSphereIntersect(Body* sphereA, Body* sphereB, contact_t& contact)
{
	assert(sphereA->GetShape()->GetType() == Shape::SHAPE_SPHERE &&
		sphereB->GetShape()->GetType() == Shape::SHAPE_SPHERE);

	ShapeSphere* shapeA = static_cast<ShapeSphere*>(sphereA->GetShape());
	ShapeSphere* shapeB = static_cast<ShapeSphere*>(sphereB->GetShape());

	Vec3 centerA = sphereA->GetCenterOfMassWorldSpace();
	Vec3 centerB = sphereB->GetCenterOfMassWorldSpace();

	Vec3 dist = centerB - centerA;

	// return dist.Dot(dist) < (shapeA->m_radius + shapeB->m_radius) * (shapeA->m_radius + shapeB->m_radius);

	// TODO: Add Code

	contact.bodyA = sphereA;
	contact.bodyB = sphereB;

	contact.normal = dist.Dir();
	contact.ptOnA_WorldSpace = sphereA->GetCenterOfMassWorldSpace() + contact.normal * shapeA->GetRadius();
	contact.ptOnB_WorldSpace = sphereB->GetCenterOfMassWorldSpace() - contact.normal * shapeB->GetRadius();

	contact.ptOnA_LocalSpace = sphereA->WorldSpacePointToLocalSpace(contact.ptOnA_WorldSpace); // GetOrientation().Inverse().RotatePoint(contact.normal * shapeA->GetRadius());
	contact.ptOnB_LocalSpace = sphereB->WorldSpacePointToLocalSpace(contact.ptOnB_WorldSpace); // m_oriGeentation.Inverse().RotatePoint(contact.normal * -shapeB->GetRadius());

	contact.separationDistance = dist.GetMagnitude() - (shapeA->GetRadius() + shapeB->GetRadius());
}

struct IntersectionFunctionTableItem
{
	EShape shapeA;
	EShape shapeB;
	void (*func)(Body* bodyA, Body* bodyB, contact_t& contact);
};

IntersectionFunctionTableItem g_intersectionFunctionTable[] =
{
	{ EShape::SHAPE_PLANE, EShape::SHAPE_PLANE, PlanePlaneIntersect },
	{ EShape::SHAPE_PLANE, EShape::SHAPE_SPHERE, PlaneShapeIntersect },
	{ EShape::SHAPE_SPHERE, EShape::SHAPE_PLANE, SpherePlaneIntersect },
	{  EShape::SHAPE_PLANE, EShape::SHAPE_PLANE, SphereSphereIntersect }
};



/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, contact_t & contact ) 
{

	EShape shapeA = bodyA->GetShape()->GetType();
	EShape shapeB = bodyB->GetShape()->GetType();

	for (int i = 0; i < p_count_of(g_intersectionFunctionTable); i++)
	{
		if (g_intersectionFunctionTable[i].shapeA == shapeA &&
			g_intersectionFunctionTable[i].shapeB == shapeB)
		{
			g_intersectionFunctionTable[i].func(bodyA, bodyB, contact);
		}
	}


	return contact.HasIntersection();
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, const float dt, contact_t & contact ) {
	// TODO: Add Code

	return false;
}






















