//
//  ConstraintPenetration.cpp
//
#include "ConstraintPenetration.h"
#include "Math/Helpers.h"

/*
================================
ConstraintPenetration::PreSolve
================================
*/

extern Vec3 g_global_gravity;

void ConstraintPenetration::PreSolve( const float dt_sec ) 
{
	// TODO: Add code
	Vec3 r1 = m_bodyA->LocalSpacePointToWorldSpace(m_anchorA) - m_bodyA->GetCenterOfMassWorldSpace();
	Vec3 r2 = m_bodyB->LocalSpacePointToWorldSpace(m_anchorB) - m_bodyB->GetCenterOfMassWorldSpace();

	// 获取两个切线方向
	Vec3 n = m_normal, t, b;
	n.GetOrtho(t, b);
	
	Vec3 r1CrossN = r1.Cross(n);
	Vec3 r2CrossN = r2.Cross(n);

	Vec3 r1CrossT = r1.Cross(t);
	Vec3 r2CrossT = r2.Cross(t);
	
	Vec3 r1CrossB = r1.Cross(b);
	Vec3 r2CrossB = r2.Cross(b);

	// 计算相交约束的雅可比矩阵
	// 计算法线方向的应力
	// (-n, -r1xn, n, r2xn)
	m_Jacobian[0][ 0] = -n.x; m_Jacobian[0][1] = -n.y; m_Jacobian[0][2] = -n.z;
	m_Jacobian[0][ 3] = -r1CrossN.x;
	m_Jacobian[0][ 4] = -r1CrossN.y;
	m_Jacobian[0][ 5] = -r1CrossN.z;
	m_Jacobian[0][ 6] =  n.x; m_Jacobian[0][7] =  n.y; m_Jacobian[0][8] =  n.z;
	m_Jacobian[0][ 9] =  r2CrossN.x;
	m_Jacobian[0][10] =  r2CrossN.y;
	m_Jacobian[0][11] =  r2CrossN.z;

	// 计算切线方向的应力
	m_Jacobian[1][ 0] = -t.x; m_Jacobian[1][1] = -t.y; m_Jacobian[1][2] = -t.z;
	m_Jacobian[1][ 3] = -r1CrossT.x;
	m_Jacobian[1][ 4] = -r1CrossT.y;
	m_Jacobian[1][ 5] = -r1CrossT.z;
	m_Jacobian[1][ 6] = t.x; m_Jacobian[1][7] = t.y; m_Jacobian[1][8] = t.z;
	m_Jacobian[1][ 9] = r2CrossT.x;
	m_Jacobian[1][10] = r2CrossT.y;
	m_Jacobian[1][11] = r2CrossT.z;

	m_Jacobian[2][ 0] = -b.x; m_Jacobian[2][1] = -b.y; m_Jacobian[2][2] = -b.z;
	m_Jacobian[2][ 3] = -r1CrossB.x;
	m_Jacobian[2][ 4] = -r1CrossB.y;
	m_Jacobian[2][ 5] = -r1CrossB.z;
	m_Jacobian[2][ 6] = b.x; m_Jacobian[2][7] = b.y; m_Jacobian[2][8] = b.z;
	m_Jacobian[2][ 9] = r2CrossB.x;
	m_Jacobian[2][10] = r2CrossB.y;
	m_Jacobian[2][11] = r2CrossB.z;


	// Warm up
	ApplyImpulses(m_Jacobian.Transpose() * m_cachedLambda);

	constexpr float beta = 0.01f;
	m_baumgarte = (r2 - r1).Dot(m_normal)  / dt_sec * beta;
}

/*
================================
ConstraintPenetration::PreSolve
================================
*/
void ConstraintPenetration::Solve() {
	// TODO: Add code
	VecN vel = GetVelocities();
	MatMN Mass = GetInverseMassMatrix();


	MatMN J_W_JT = m_Jacobian * Mass * m_Jacobian.Transpose();
	// 由于baugarte参数只计算了法线方向的差值，因此只在法线分量上减去参数。
	VecN rhs = (m_Jacobian * vel) * -1.0f;

	rhs[0] -= m_baumgarte;

	VecN lambda = LCP_GaussSeidel(J_W_JT, rhs);
	// cache上一帧求解的lambda

	VecN nextLambda = lambda + m_cachedLambda;
	// 保证法线方向的力永远是朝外的
	// nextLambda[0] = Max(nextLambda[0], 0.0f);

	
	if (m_friction > 0.0f)
	{
		float maxFriction = Abs(nextLambda[0] * m_friction);//(g_global_gravity * m_friction).Dot(m_normal);

		nextLambda[1] = Clamp(nextLambda[1], -maxFriction, maxFriction);
		nextLambda[2] = Clamp(nextLambda[2], -maxFriction, maxFriction);
	}
	

	ApplyImpulses(m_Jacobian.Transpose()* (nextLambda - m_cachedLambda));
	m_cachedLambda = nextLambda;
}

void ConstraintPenetration::SetNormal(Vec3 normal)
{
	m_normal = normal.Dir();
}

void ConstraintPenetration::SetFriction(float friction)
{
	m_friction = friction;
}
