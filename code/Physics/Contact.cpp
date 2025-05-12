//
//  Contact.cpp
//
#include "Contact.h"

/*
====================================================
ResolveContact
====================================================
*/
void ResolveContact( contact_t & contact ) 
{
	assert(contact.HasIntersection());

	// TODO: Add Code
	Body* bodyA = contact.bodyA;
	Body* bodyB = contact.bodyB;

	const Vec3& n = contact.normal;

	/*
	相关公式见https://gamephysicsweekend.github.io/pdfs/GamePhysicsInOneWeekend.pdf，p.50
	*/
	Vec3 rA = contact.ptOnA_WorldSpace - bodyA->GetCenterOfMassWorldSpace();
	Vec3 rB = contact.ptOnB_WorldSpace - bodyB->GetCenterOfMassWorldSpace();

	// 该点线速度
	Vec3 velA = bodyA->GetLinearVelocity() + bodyA->GetAngularVelocity().Cross(rA);
	Vec3 velB = bodyB->GetLinearVelocity() + bodyB->GetAngularVelocity().Cross(rB);

	Vec3 velBA = velB - velA;
	Vec3 velNormal = n * n.Dot(velBA);
	Vec3 velTangent = velBA - velNormal;


	Vec3 angularFactorA = bodyA->GetInverseInertiaTensorWorldSpace() * (rA.Cross(n)).Cross(rA);
	Vec3 angularFactorB = bodyB->GetInverseInertiaTensorWorldSpace() * (rB.Cross(n)).Cross(rB);
	float angularFactor = (angularFactorA + angularFactorB).Dot(n);

	float e = bodyA->GetElasity() * bodyB->GetElasity();

	Vec3 JA = velNormal * (1 + e)  / (bodyA->GetInvMass() + bodyB->GetInvMass() - angularFactor);
	Vec3 JB = JA * -1;

	bodyA->ApplyImpulse(JA);
	bodyB->ApplyImpulse(JB);

	// 摩擦力
	Vec3 tangent = velTangent.Dir();
	
	float mu = bodyA->GetFriction() * bodyB->GetFriction();
	// 摩擦力带来的冲量
	Vec3 inertiaA = bodyA->GetInverseInertiaTensorWorldSpace() * rA.Cross(tangent).Cross(rA);
	Vec3 inertiaB = bodyB->GetInverseInertiaTensorWorldSpace() * rB.Cross(tangent).Cross(rB);
	float inertiaFric = (inertiaA + inertiaB).Dot(tangent);

	Vec3 JfA = velTangent * mu / (bodyA->GetInvMass() + bodyB->GetInvMass() + inertiaFric);
	Vec3 JfB = JfA * -1;

	bodyA->ApplyImpulse(JfA, contact.ptOnA_WorldSpace);
	bodyB->ApplyImpulse(JfB, contact.ptOnB_WorldSpace);

	// 利用重心不变修正位置
	Vec3 dist = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;

	bodyA->FixPosition(dist * bodyA->GetInvMass() / (bodyA->GetInvMass() + bodyB->GetInvMass()));
	bodyB->FixPosition(dist * -bodyB->GetInvMass() / (bodyA->GetInvMass() + bodyB->GetInvMass()));

}