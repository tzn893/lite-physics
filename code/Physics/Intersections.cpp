//
//  Intersections.cpp
//
#include "Intersections.h"
#include "GJK.h"
#include "SAT.h"
#include "Math/Helpers.h"
#include <functional>

#include <array>

#define p_count_of(arr) (sizeof(arr) / sizeof(arr[0]))


// 对于两个box，使用SAT进行碰撞检测
void BoxBoxIntersect(Body* boxA, Body* boxB, IntersectionManifold& manifold)
{
	// TODO 重构GJK，将其返回值设置为manifold
	manifold.contactCount = 1;
	contact_t& contact = manifold.contacts[0];

	contact.bodyA = boxA;
	contact.bodyB = boxB;

	SATSolver(boxA, boxB).HasIntersection(contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace,
		contact.separationDistance, contact.normal);

	contact.ptOnA_LocalSpace = boxA->WorldSpacePointToLocalSpace(contact.ptOnA_WorldSpace);
	contact.ptOnB_LocalSpace = boxB->WorldSpacePointToLocalSpace(contact.ptOnB_WorldSpace);
	contact.timeOfImpact = 0.0f;
}

// 对于球体和方形的碰撞检测
void SphereBoxIntersect(Body* sphereA, Body* boxB, IntersectionManifold& manifold)
{
	assert(sphereA->GetShape()->GetType() == EShape::SHAPE_SPHERE &&
		boxB->GetShape()->GetType() == EShape::SHAPE_BOX);

	ShapeSphere* sphereAShape = dynamic_cast<ShapeSphere*>(sphereA->GetShape());
	ShapeBox* boxBShape = dynamic_cast<ShapeBox*>(boxB->GetShape());

	float sphereRadius = sphereAShape->GetRadius();
	Vec3 boxHalfExtent = Vec3(boxBShape->GetWidth(), boxBShape->GetLength(), boxBShape->GetHeight()) / 2.0f; 

	// 首先将球体的中心点转换到方形的局部空间
	Vec3 boxCenter = boxB->GetCenterOfMassWorldSpace();
	Quat boxOrient = boxB->GetOrientation();

	// 转换到方形的局部空间下球体的坐标
	Vec3 sphereCenter = boxOrient.Inverse().RotatePoint(sphereA->GetCenterOfMassWorldSpace() - boxCenter);
	
	Vec3 closestPointOnBox, closestPointOnSphere;
	bool intersect = false;

	// 球的中心在方形内部
	if (Abs(sphereCenter.x) < boxHalfExtent.x && Abs(sphereCenter.y) < boxHalfExtent.y 
		&& Abs(sphereCenter.z) < boxHalfExtent.z)
	{
		intersect = true;

		// 找到方形边缘距离圆心最近的点以及对应的方向
		Vec3 closestPt;
		float closestDistance = sphereRadius;
		int closestAxis = 0;

		int axisDimension[] = { 0, 0, 1, 1, 2, 2 };
		int axisSign[] = {1, -1, 1, -1, 1, -1};
		Vec3 axisDir[] = { Vec3( 1, 0, 0), Vec3(-1, 0, 0), Vec3( 0, 1, 0), 
			Vec3( 0,-1, 0), Vec3( 0, 0, 1), Vec3( 0, 0,-1) };

		for (int i = 0; i < 6; i++)
		{
			float axisDistance = Abs(sphereCenter[axisDimension[i]] - boxHalfExtent[axisDimension[i]] * axisSign[i]);
			if (axisDistance < closestDistance)
			{
				closestAxis = i;
				closestDistance = axisDistance;
				closestPt = sphereCenter;
				closestPt[closestAxis] = boxHalfExtent[axisDimension[i]] * axisSign[i];
			}
		}

		closestPointOnSphere = sphereCenter + axisDir[closestAxis] * -1 * sphereRadius;
		closestPointOnBox = closestPt;
	}
	else
	{
		closestPointOnBox = Clamp(sphereCenter,
			Vec3(-boxHalfExtent.x, -boxHalfExtent.y, -boxHalfExtent.z),
			Vec3(boxHalfExtent.x, boxHalfExtent.y, boxHalfExtent.z));
		closestPointOnSphere = (closestPointOnBox - sphereCenter).Dir() * sphereRadius +
			sphereCenter;

		intersect = (sphereCenter - closestPointOnBox).GetLengthSqr() < sphereRadius * sphereRadius;
	}

	manifold.contactCount = 1;
	contact_t& contact = manifold.contacts[0];

	contact.ptOnB_WorldSpace = boxOrient.RotatePoint(closestPointOnBox) + boxCenter;
	contact.ptOnB_LocalSpace = boxB->WorldSpacePointToLocalSpace(contact.ptOnB_WorldSpace);
	contact.ptOnA_WorldSpace = boxOrient.RotatePoint(closestPointOnSphere) + boxCenter;
	contact.ptOnA_LocalSpace = sphereA->WorldSpacePointToLocalSpace(contact.ptOnA_WorldSpace);

	contact.bodyA = sphereA;
	contact.bodyB = boxB;

	contact.normal = (contact.ptOnA_WorldSpace - contact.ptOnB_WorldSpace).Dir();
	contact.separationDistance = (contact.ptOnA_WorldSpace - contact.ptOnB_WorldSpace).GetMagnitude() * 
		(intersect ? -1 : 1);
	contact.timeOfImpact = 0.0f;
}

void BoxSphereIntersect(Body* boxA, Body* sphereB, IntersectionManifold& contact)
{
	SphereBoxIntersect(sphereB, boxA, contact);
}


void PlanePlaneIntersect(Body* planeA, Body* planeB, IntersectionManifold& contact)
{
	// TODO
	assert(false);
}



void SpherePlaneIntersect(Body* sphereA, Body* planeB, IntersectionManifold& manifold)
{
	assert(sphereA->GetShape()->GetType() == Shape::SHAPE_SPHERE &&
		planeB->GetShape()->GetType() == Shape::SHAPE_PLANE);

	ShapeSphere* shapeAShape = static_cast<ShapeSphere*>(sphereA->GetShape());
	ShapePlane* planeBShape = static_cast<ShapePlane*>(planeB->GetShape());

	Vec3 planeNormal = planeBShape->GetNormalWorldSpace(planeB->GetOrientation());
	Vec3 planePos = planeB->GetBodyPositionWorldSpace();
	Vec3 spherePos = sphereA->GetBodyPositionWorldSpace();

	//// 平面方程为 Ax+By+Cz+D=0, 其中A,B,C为planeNormal的三个分量
	float D = -planeNormal.Dot(planePos);
	float offset = planeNormal.Dot(spherePos) + D;
	
	float dist = Abs(offset) / Sqrt(planeNormal.Dot(planeNormal));

	// 无限大的平面
	manifold.contactCount = 1;
	contact_t& contact = manifold.contacts[0];

	contact.bodyA = sphereA;
	contact.bodyB = planeB;

	contact.normal =  planeNormal * Sign(offset);
	// contact.ptOnA_WorldSpace = spherePos - contact.normal * shapeAShape->GetRadius();
	// contact.ptOnA_LocalSpace = sphereA->WorldSpacePointToLocalSpace(contact.ptOnA_WorldSpace);

	Vec3 planePtWorld = spherePos - contact.normal * dist;
	Vec3 planePtLocal = planeB->WorldSpacePointToLocalSpace(planePtWorld);

	// 在平面的local space计算圆与有限平面的相交
	// 可以分为三种情况 
	// 1.圆心在平面上的投影点在有限平面内，圆的最底部点与平面相交，退化为无限大的平面相交。
	// 2.圆心在平面外。过圆心对平面四条边对应的直线做垂线，若存在某点处于对应边的线段内，则该点为圆与平面交点。
	// 3.圆心在平面外。过圆心对平面四条边对应的直线做垂线，且所有点都在对应线段外，则离圆心最近的点为圆与平面交点。
	// 情况2.,3.对应点的坐标可以直接使用Clamp将接触点范围限制在平面内直接计算。

	Vec2 planeExtent = planeBShape->GetExtent();

	// 得到修正后的交点位置
	planePtLocal = Clamp(planePtLocal,  
		Vec3(- planeExtent.x / 2.0f, - planeExtent.y / 2.0f, planePtLocal.z),
		Vec3(planeExtent.x / 2.0f, planeExtent.y / 2.0f, planePtLocal.z)
	);
	// 将交点转到世界空间
	planePtWorld = planeB->LocalSpacePointToWorldSpace(planePtLocal);

	contact.normal = (spherePos - planePtWorld).Dir();
	contact.ptOnA_WorldSpace = spherePos - contact.normal * shapeAShape->GetRadius();
	contact.ptOnA_LocalSpace = sphereA->WorldSpacePointToLocalSpace(contact.ptOnA_WorldSpace);

	contact.ptOnB_LocalSpace = planePtLocal;
	contact.ptOnB_WorldSpace = planePtWorld;

	float fixedDist = (spherePos - planePtWorld).GetMagnitude();

	contact.separationDistance = fixedDist - shapeAShape->GetRadius();
}

void PlaneShapeIntersect(Body* planeB, Body* sphereA, IntersectionManifold& manifold)
{
	SpherePlaneIntersect(sphereA, planeB, manifold);
}

void SphereSphereIntersect(Body* sphereA, Body* sphereB, IntersectionManifold& manifold)
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
	manifold.contactCount = 1;
	contact_t& contact = manifold.contacts[0];

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

// 对于任意形状，使用GJK算法做碰撞检测
void GeneralIntersect(Body* bodyA, Body* bodyB, IntersectionManifold& manifold)
{
	/*
	manifold.contactCount = 1;
	contact_t& contact = manifold.contacts[0];
	*/

	Vec3 ptOnA, ptOnB;
	bool hasIntersection = GJK_DoesIntersect(bodyA, bodyB, 1e-4f, ptOnA, ptOnB);

	if(hasIntersection)
	{
		ClippingManifold(bodyA, bodyB, ptOnA, ptOnB, manifold);
	}
	
	/*
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;
	contact.normal = (contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace).Dir() * -1;
	contact.ptOnA_LocalSpace = bodyA->WorldSpacePointToLocalSpace(contact.ptOnA_WorldSpace);
	contact.ptOnB_LocalSpace = bodyB->WorldSpacePointToLocalSpace(contact.ptOnB_WorldSpace);

	contact.separationDistance = (contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace).GetMagnitude() * 
		(hasIntersection ? -1 : 1);

	contact.timeOfImpact = 0;
	*/
}

using IntersectionFunc = std::function<void(Body* bodyA, Body* bodyB, IntersectionManifold& contact)>;


struct IntersectionFunctionTableItem
{
	EShape shapeA;
	EShape shapeB;
	IntersectionFunc func;
};

IntersectionFunctionTableItem g_intersectionFunctionTable[] =
{
	{ EShape::SHAPE_PLANE	, EShape::SHAPE_PLANE	, PlanePlaneIntersect	},
	{ EShape::SHAPE_PLANE	, EShape::SHAPE_SPHERE	, PlaneShapeIntersect	},
	{ EShape::SHAPE_SPHERE	, EShape::SHAPE_PLANE	, SpherePlaneIntersect	},
	{ EShape::SHAPE_SPHERE	, EShape::SHAPE_SPHERE	, SphereSphereIntersect	},
	{ EShape::SHAPE_SPHERE	, EShape::SHAPE_BOX		, SphereBoxIntersect	},
	{ EShape::SHAPE_BOX		, EShape::SHAPE_SPHERE	, BoxSphereIntersect	},
	// { EShape::SHAPE_BOX		, EShape::SHAPE_BOX		, BoxBoxIntersect		}

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

	return GeneralIntersect;
}

using CCDSolver = std::function<void(Body* bodyA, Body* bodyB, IntersectionManifold& manifold, float dt)>;

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

void GeneralCCDSolver(Body* bodyA, Body* bodyB, IntersectionManifold& manifold, IntersectionFunc intersectionFunc, float dt)
{
	 // TODO 目前还未实现通用的CCD
	assert(false);
}


void SpherePlaneCCDSolver(Body* sphereA, Body* planeB, IntersectionManifold& manifold, float dt)
{
	// TODO 目前直接对球体和平面做相交测试，不进行任何CCD计算
	SpherePlaneIntersect(sphereA, planeB, manifold);
}

void PlaneSphereCCDSolver(Body* planeA, Body* sphereB, IntersectionManifold& manifold, float dt)
{
	SpherePlaneCCDSolver(sphereB, planeA, manifold, dt);
}

void SphereSphereCCDSolver(Body* sphereA, Body* sphereB, IntersectionManifold& manifold, float dt)
{
	

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
	// 如果相对速度小于最小的隧穿速度，则不需要进行CCD
	if (relativeVel.Dot(relativeVel) < minCCDVel * minCCDVel)
	{
		SphereSphereIntersect(sphereA, sphereB, manifold);
		return;
	}

	manifold.contactCount = 1;
	contact_t& contact = manifold.contacts[0];

	contact.bodyA = sphereA;
	contact.bodyB = sphereB;

	// 做球体射线检测，判断两球相交时间
	float t0, t1;
	Vec3 relativeTravelDistance = relativeVel * dt;
	if (SphereRayIntersection(spherePosB, sphereAShape->GetRadius() + sphereBShape->GetRadius(),
		spherePosA, relativeDir, t0, t1))
	{
		// 若相交，t0为最早相交的时间点，
		t0 = Max(t0 , 0.0f);
		// 只有t0小于dt时，两物体才会在当前帧相交
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

	SphereSphereIntersect(sphereA, sphereB, manifold);
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
		return [func](Body* bodyA, Body* bodyB, IntersectionManifold& manifold, float dt)
		{
				GeneralCCDSolver(bodyA, bodyB, manifold, func, dt);
		};
	}

	return nullptr;
}


/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, IntersectionManifold& manifold ) 
{
	bool useCCD = bodyA->CCDEnabled() || bodyB->CCDEnabled();

	if (!useCCD)
	{
		if (IntersectionFunc func = FindIntersectionFunction(bodyA, bodyB); func != nullptr)
		{
			func(bodyA, bodyB, manifold);
		}
	}
	
	return manifold.HasIntersection();
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, const float dt, IntersectionManifold& manifold) {
	// TODO: Add Code
	bool useCCD = bodyA->CCDEnabled() || bodyB->CCDEnabled();

	if (!useCCD)
	{
		if (IntersectionFunc func = FindIntersectionFunction(bodyA, bodyB); func != nullptr)
		{
			func(bodyA, bodyB, manifold);
		}
	}
	else
	{
		if (CCDSolver solver = FindCCDSolver(bodyA, bodyB); solver != nullptr)
		{
			solver(bodyA, bodyB, manifold, dt);
		}
	}

	return manifold.HasIntersection();
}

IntersectionManifold::IntersectionManifold()
{
	contactCount = 0;
	for (int i = 0; i < maxContactCount; i++)
	{
		contacts[i].timeOfImpact = 0.0f;
		contacts[i].separationDistance = 0.0f;
	}
}

IntersectionManifold::IntersectionManifold(const IntersectionManifold& rhs)
{
	contactCount = rhs.contactCount;
	for (int i = 0; i < maxContactCount; i++)
	{
		contacts[i] = rhs.contacts[i];
	}
}

bool IntersectionManifold::HasIntersection()
{
	return contactCount > 1 || (contactCount == 1 && contacts[0].HasIntersection());
}
