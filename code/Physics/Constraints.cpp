//
//  Constraints.cpp
//
#include "Constraints.h"


/*
====================================================
Constraint::GetInverseMassMatrix
====================================================
*/
MatMN Constraint::GetInverseMassMatrix() const
{
	MatMN invMassMatrix(12, 12);

	// TODO: Add code
	Mat3 invTensorA = m_bodyA->GetInertialTensorWorldSpace();
	Mat3 invTensorB = m_bodyB->GetInertialTensorWorldSpace();
	float massA = m_bodyA->GetInvMass();
	float massB = m_bodyB->GetInvMass();

	invMassMatrix[0][0] = massA;
	invMassMatrix[1][1] = massA;
	invMassMatrix[2][2] = massA;

	for (int y = 0; y < 3; y++)
	{
		for (int x = 0; x < 3; x++)
		{
			invMassMatrix[x + 3][y + 3] = invTensorA.rows[x][y];
		}
	}

	invMassMatrix[7][7] = massB;
	invMassMatrix[8][8] = massB;
	invMassMatrix[9][9] = massB;


	for (int y = 0; y < 3; y++)
	{
		for (int x = 0; x < 3; x++)
		{
			invMassMatrix[x + 9][y + 9] = invTensorB.rows[x][y];
		}
	}

	return invMassMatrix;
}


/*
====================================================
Constraint::GetVelocities
====================================================
*/
VecN Constraint::GetVelocities() const {
	VecN q_dt(12);

	// TODO: Add code
	Vec3 velA = m_bodyA->GetLinearVelocity();
	Vec3 angVelA = m_bodyA->GetAngularVelocity();
	Vec3 velB = m_bodyB->GetLinearVelocity();
	Vec3 angVelB = m_bodyB->GetAngularVelocity();

	
	q_dt[0] = velA[0];
	q_dt[1] = velA[1];
	q_dt[2] = velA[2];

	q_dt[3] = angVelA[0];
	q_dt[4] = angVelA[1];
	q_dt[5] = angVelA[2];

	q_dt[6] = velB[0];
	q_dt[7] = velB[1];
	q_dt[8] = velB[2];


	q_dt[ 9] = angVelB[0];
	q_dt[10] = angVelB[1];
	q_dt[11] = angVelB[2];
	

	// FillVectorN(q_dt, velA, angVelA, velB, angVelB);

	return q_dt;
}

/*
====================================================
Constraint::ApplyImpulses
====================================================
*/
void Constraint::ApplyImpulses(const VecN& impulses) {
	// TODO: Add code

	Vec3 impulseA(impulses[0], impulses[1], impulses[2]);
	Vec3 torqueA(impulses[3], impulses[4], impulses[5]);
	Vec3 impulseB(impulses[6], impulses[7], impulses[8]);
	Vec3 torqueB(impulses[9], impulses[10], impulses[11]);

	m_bodyA->ApplyImpulse(impulseA);
	m_bodyA->ApplyTorch(torqueA);

	m_bodyB->ApplyImpulse(impulseB);
	m_bodyB->ApplyTorch(torqueB);
}

/*
====================================================
Constraint::Left
====================================================
*/
Mat4 Constraint::Left(const Quat& q) {
	Mat4 L;

	// TODO: Add code

	return L.Transpose();
}

/*
====================================================
Constraint::Right
====================================================
*/
Mat4 Constraint::Right(const Quat& q) {
	Mat4 R;

	// TODO: Add code

	return R.Transpose();
}

void Constraint::SetBodies(Body* bodyA, Vec3 anchorA, Vec3 axisA, Body* bodyB, Vec3 anchorB, Vec3 axisB)
{
	m_bodyA = bodyA;
	m_bodyB = bodyB;

	m_anchorA = anchorA;
	m_anchorB = anchorB;

	m_axisA = axisA;
	m_axisB = axisB;
}
