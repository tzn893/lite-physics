//
//  ConstraintDistance.cpp
//
#include "ConstraintDistance.h"
#include "Math/Helpers.h"

void ConstraintDistance::SetBodies(Body* bodyA, Vec3 anchorA, Vec3 axisA, Body* bodyB, Vec3 anchorB, Vec3 axisB)
{
	Constraint::SetBodies(bodyA, anchorA, axisA, bodyB, anchorB, axisB);

	Vec3 r1 = m_bodyA->LocalSpacePointToWorldSpace(m_anchorA);
	Vec3 r2 = m_bodyB->LocalSpacePointToWorldSpace(m_anchorB);

	m_originalDistance = (r2 - r1).GetLengthSqr();
}

// 计算对应jacobian
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

	// 热启动
	ApplyImpulses(m_Jacobian * m_cachedLambda);

	// 计算baumgarte参数
	constexpr float beta = 0.05f;
	m_baumgarte = ((r2 - r1).GetLengthSqr() - m_originalDistance) / dt_sec * beta;
}

void ConstraintDistance::Solve()
{
	VecN vel = GetVelocities();
	MatMN Mass = GetInverseMassMatrix();

	float lambda = (m_Jacobian.Dot(vel) + m_baumgarte) / (m_Jacobian.Dot(Mass * m_Jacobian) + 1e-8f) * -1.f;

	ApplyImpulses(m_Jacobian * lambda);

	// cache上一帧求解的lambda
	m_cachedLambda += lambda;
}

void ConstraintDistance::PostSolve() 
{
	// TODO: Add code
	if (m_cachedLambda * 0.0f != m_cachedLambda * 0.0f)
	{
		m_cachedLambda = 0.0f;
	}

	float infinity = 1e15f;
	m_cachedLambda = Clamp(m_cachedLambda, -infinity, infinity);
}