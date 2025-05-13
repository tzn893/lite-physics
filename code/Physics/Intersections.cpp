//
//  Intersections.cpp
//
#include "Intersections.h"
#include "GJK.h"
#include "Math/Helpers.h"
#include <functional>

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

	ShapeSphere* shapeAShape = static_cast<ShapeSphere*>(sphereA->GetShape());
	ShapePlane* planeBShape = static_cast<ShapePlane*>(planeB->GetShape());

	Vec3 planeNormal = planeBShape->GetNormalWorldSpace(planeB->GetOrientation());
	Vec3 planePos = planeB->GetBodyPositionWorldSpace();
	Vec3 spherePos = sphereA->GetBodyPositionWorldSpace();

	//// ƽ�淽��Ϊ Ax+By+Cz+D=0, ����A,B,CΪplaneNormal����������
	float D = -planeNormal.Dot(planePos);
	float offset = planeNormal.Dot(spherePos) + D;
	
	float dist = Abs(offset) / Sqrt(planeNormal.Dot(planeNormal));

	// ���޴��ƽ��
	contact.bodyA = sphereA;
	contact.bodyB = planeB;

	contact.normal =  planeNormal * Sign(offset);
	// contact.ptOnA_WorldSpace = spherePos - contact.normal * shapeAShape->GetRadius();
	// contact.ptOnA_LocalSpace = sphereA->WorldSpacePointToLocalSpace(contact.ptOnA_WorldSpace);

	Vec3 planePtWorld = spherePos - contact.normal * dist;
	Vec3 planePtLocal = planeB->WorldSpacePointToLocalSpace(planePtWorld);

	// ��ƽ���local space����Բ������ƽ����ཻ
	// ���Է�Ϊ������� 
	// 1.Բ����ƽ���ϵ�ͶӰ��������ƽ���ڣ�Բ����ײ�����ƽ���ཻ���˻�Ϊ���޴��ƽ���ཻ��
	// 2.Բ����ƽ���⡣��Բ�Ķ�ƽ�������߶�Ӧ��ֱ�������ߣ�������ĳ�㴦�ڶ�Ӧ�ߵ��߶��ڣ���õ�ΪԲ��ƽ�潻�㡣
	// 3.Բ����ƽ���⡣��Բ�Ķ�ƽ�������߶�Ӧ��ֱ�������ߣ������е㶼�ڶ�Ӧ�߶��⣬����Բ������ĵ�ΪԲ��ƽ�潻�㡣
	// ���2.,3.��Ӧ����������ֱ��ʹ��Clamp���Ӵ��㷶Χ������ƽ����ֱ�Ӽ��㡣

	Vec2 planeExtent = planeBShape->GetExtent();

	// �õ�������Ľ���λ��
	planePtLocal = Clamp(planePtLocal,  
		Vec3(- planeExtent.x / 2.0f, - planeExtent.y / 2.0f, planePtLocal.z),
		Vec3(planeExtent.x / 2.0f, planeExtent.y / 2.0f, planePtLocal.z)
	);
	// ������ת������ռ�
	planePtWorld = planeB->LocalSpacePointToWorldSpace(planePtLocal);

	contact.normal = (spherePos - planePtWorld).Dir();
	contact.ptOnA_WorldSpace = spherePos - contact.normal * shapeAShape->GetRadius();
	contact.ptOnA_LocalSpace = sphereA->WorldSpacePointToLocalSpace(contact.ptOnA_WorldSpace);

	contact.ptOnB_LocalSpace = planePtLocal;
	contact.ptOnB_WorldSpace = planePtWorld;

	float fixedDist = (spherePos - planePtWorld).GetMagnitude();

	contact.separationDistance = fixedDist - shapeAShape->GetRadius();
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
	contact.timeOfImpact = 0;
}

using IntersectionFunc = std::function<void(Body* bodyA, Body* bodyB, contact_t& contact)>;


struct IntersectionFunctionTableItem
{
	EShape shapeA;
	EShape shapeB;
	IntersectionFunc func;
};

IntersectionFunctionTableItem g_intersectionFunctionTable[] =
{
	{ EShape::SHAPE_PLANE, EShape::SHAPE_PLANE, PlanePlaneIntersect },
	{ EShape::SHAPE_PLANE, EShape::SHAPE_SPHERE, PlaneShapeIntersect },
	{ EShape::SHAPE_SPHERE, EShape::SHAPE_PLANE, SpherePlaneIntersect },
	{  EShape::SHAPE_SPHERE, EShape::SHAPE_SPHERE, SphereSphereIntersect }
};

IntersectionFunc FindIntersectionFunction(Body* bodyA, Body* bodyB)
{
	EShape shapeA = bodyA->GetShape()->GetType();
	EShape shapeB = bodyB->GetShape()->GetType();

	for (int i = 0; i < p_count_of(g_intersectionFunctionTable); i++)
	{
		if (g_intersectionFunctionTable[i].shapeA == shapeA &&
			g_intersectionFunctionTable[i].shapeB == shapeB)
		{
			return g_intersectionFunctionTable[i].func;
		}
	}

	return nullptr;
}

using CCDSolver = std::function<void(Body* bodyA, Body* bodyB, contact_t& contact, float dt)>;

struct CCDSolverTableItem
{
	EShape shapeA;
	EShape shapeB;
	CCDSolver func;//void (*func)(Body* bodyA, Body* bodyB, contact_t& contact);
};


float GetThickness(Body* bodyA, Vec3 dir)
{
	Bounds bound = bodyA->GetBounds();
	float t1, t0;
    AABBRayIntersection(bound.mins, bound.maxs, bodyA->GetCenterOfMassWorldSpace(), dir, t0, t1);

	return t1 - t0;
}

void GeneralCCDSolver(Body* bodyA, Body* bodyB, contact_t& contact, IntersectionFunc intersectionFunc, float dt)
{
	 // TODO 
}


void SpherePlaneCCDSolver(Body* sphereA, Body* planeB, contact_t& contact, float dt)
{
	// TODO Ŀǰֱ�Ӷ������ƽ�����ཻ���ԣ��������κ�CCD����
	SpherePlaneIntersect(sphereA, planeB, contact);
}

void PlaneSphereCCDSolver(Body* planeA, Body* sphereB, contact_t& contact, float dt)
{
	SpherePlaneCCDSolver(sphereB, planeA, contact, dt);
}

void SphereSphereCCDSolver(Body* sphereA, Body* sphereB, contact_t& contact, float dt)
{
	contact.bodyA = sphereA;
	contact.bodyB = sphereB;

	// TODO
	ShapeSphere* sphereAShape = static_cast<ShapeSphere*>(sphereA->GetShape());
	ShapeSphere* sphereBShape = static_cast<ShapeSphere*>(sphereB->GetShape());

	Vec3 spherePosA = sphereA->GetBodyPositionWorldSpace();
	Vec3 spherePosB = sphereB->GetBodyPositionWorldSpace();

	Vec3 sphereVelA = sphereA->GetLinearVelocity();
	Vec3 sphereVelB = sphereB->GetLinearVelocity();

	Vec3 relativeVel = sphereVelA - sphereVelB;
	Vec3 relativeDir = relativeVel.Dir();

	float minCCDVel = (sphereAShape->GetRadius() + sphereBShape->GetRadius()) / dt;
	// �������ٶ�С����С�����ٶȣ�����Ҫ����CCD
	if (relativeVel.Dot(relativeVel) < minCCDVel * minCCDVel)
	{
		SphereSphereIntersect(sphereA, sphereB, contact);
		return;
	}

	// ���������߼�⣬�ж������ཻʱ��
	float t0, t1;
	Vec3 relativeTravelDistance = relativeVel * dt;
	if (SphereRayIntersection(spherePosB, sphereAShape->GetRadius() + sphereBShape->GetRadius(),
		spherePosA, relativeDir, t0, t1))
	{
		// ���ཻ��t0Ϊ�����ཻ��ʱ��㣬
		t0 = Max(t0 , 0.0f);
		// ֻ��t0С��dtʱ��������Ż��ڵ�ǰ֡�ཻ
		if (t0 < relativeVel.GetMagnitude() * dt)
		{
			float timeOfImpact = t0 / relativeVel.GetMagnitude();
			Vec3 newSpherePosA = spherePosA + sphereVelA * timeOfImpact;// sphereA->GetBodyPositionWorldSpace();
			Vec3 newSpherePosB = spherePosB + sphereVelB * timeOfImpact; // sphereB->GetBodyPositionWorldSpace();

			Vec3 dangleA = sphereA->GetAngularVelocity() * timeOfImpact;
			Quat newSphereOrientA = Quat(dangleA, dangleA.GetMagnitude()) * sphereA->GetOrientation();
			
			Vec3 dangleB = sphereB->GetAngularVelocity() * timeOfImpact;
			Quat newSphereOrientB = Quat(dangleB, dangleB.GetMagnitude()) * sphereB->GetOrientation();

			Vec3 newPosDist = newSpherePosB - newSpherePosA;

			contact.normal = newPosDist.Dir();

			contact.ptOnA_WorldSpace = newSpherePosA + contact.normal * sphereAShape->GetRadius();
			contact.ptOnB_WorldSpace = newSpherePosB - contact.normal * sphereBShape->GetRadius();

			contact.ptOnA_LocalSpace = newSphereOrientA.Inverse().RotatePoint(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = newSphereOrientB.Inverse().RotatePoint(contact.ptOnB_WorldSpace);

			contact.timeOfImpact = timeOfImpact;
			contact.separationDistance = newPosDist.GetMagnitude() - (sphereAShape->GetRadius() + sphereBShape->GetRadius());

			return;
		}
	}

	SphereSphereIntersect(sphereA, sphereB, contact);
}

CCDSolverTableItem g_CCDSolverFunctionTable[] =
{
	{EShape::SHAPE_SPHERE, EShape::SHAPE_SPHERE, SphereSphereCCDSolver},
	{EShape::SHAPE_SPHERE, EShape::SHAPE_PLANE, SpherePlaneCCDSolver},
	{EShape::SHAPE_PLANE, EShape::SHAPE_SPHERE, PlaneSphereCCDSolver}
};


CCDSolver FindCCDSolver(Body* bodyA, Body* bodyB)
{
	EShape shapeA = bodyA->GetShape()->GetType();
	EShape shapeB = bodyB->GetShape()->GetType();

	for (int i = 0; i < p_count_of(g_CCDSolverFunctionTable); i++)
	{
		if (g_CCDSolverFunctionTable[i].shapeA == shapeA &&
			g_CCDSolverFunctionTable[i].shapeB == shapeB)
		{
			return g_CCDSolverFunctionTable[i].func;
		}
	}

	if (IntersectionFunc func = FindIntersectionFunction(bodyA, bodyB); func != nullptr)
	{
		return [func](Body* bodyA, Body* bodyB, contact_t& contact, float dt)
		{
				GeneralCCDSolver(bodyA, bodyB, contact, func, dt);
		};
	}

	return nullptr;
}


/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, contact_t & contact ) 
{
	bool useCCD = bodyA->CCDEnabled() || bodyB->CCDEnabled();

	if (!useCCD)
	{
		if (IntersectionFunc func = FindIntersectionFunction(bodyA, bodyB); func != nullptr)
		{
			func(bodyA, bodyB, contact);
		}
	}
	
	return contact.HasIntersection();
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, const float dt, contact_t & contact) {
	// TODO: Add Code
	bool useCCD = bodyA->CCDEnabled() || bodyB->CCDEnabled();

	if (!useCCD)
	{
		if (IntersectionFunc func = FindIntersectionFunction(bodyA, bodyB); func != nullptr)
		{
			func(bodyA, bodyB, contact);
		}
	}
	else
	{
		if (CCDSolver solver = FindCCDSolver(bodyA, bodyB); solver != nullptr)
		{
			solver(bodyA, bodyB, contact, dt);
		}
	}

	return contact.HasIntersection();
}






















