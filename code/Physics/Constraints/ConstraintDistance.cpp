//
//  ConstraintDistance.cpp
//
#include "ConstraintDistance.h"

// ¼ÆËã¶ÔÓ¦jacobian
void ConstraintDistance::PreSolve( const float dt_sec ) 
{
	Vec3 r1 = m_bodyA->LocalSpacePointToWorldSpace(m_anchorA);
	Vec3 r2 = m_bodyB->LocalSpacePointToWorldSpace(m_anchorB);

	Vec3 rA = r1 - m_bodyA->GetCenterOfMassWorldSpace();
	Vec3 rB = r2 - m_bodyB->GetCenterOfMassWorldSpace();

	Vec3 JVelA = (r1 - r2) * 2.0f;
	Vec3 JAngVelA = rA.Cross(r1 - r2) * 2.0f;
	Vec3 JVelB = (r2 - r1) * 2.0f;
	Vec3 JAngVelB = rB.Cross(r2 - r1) * 2.0f;

	m_Jacobian[0] = JVelA[0];
	m_Jacobian[1] = JVelA[1];
	m_Jacobian[2] = JVelA[2];

	m_Jacobian[3] = JAngVelA[0];
	m_Jacobian[4] = JAngVelA[1];
	m_Jacobian[5] = JAngVelA[2];

	m_Jacobian[6] = JVelB[0];
	m_Jacobian[7] = JVelB[1];
	m_Jacobian[8] = JVelB[2];

	m_Jacobian[ 9] = JAngVelB[0];
	m_Jacobian[10] = JAngVelB[1];
	m_Jacobian[11] = JAngVelB[2];

	// FillVectorN(m_Jacobian, JVelA, JAngVelA, JVelB, JAngVelB);
}

void ConstraintDistance::Solve()
{
	VecN vel = GetVelocities();
	MatMN Mass = GetInverseMassMatrix();

	float lambda = m_Jacobian.Dot(vel) / (m_Jacobian.Dot(Mass * m_Jacobian));

	ApplyImpulses(m_Jacobian * lambda);
}

void ConstraintDistance::PostSolve() 
{
	// TODO: Add code
}