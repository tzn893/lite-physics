//
//	ConstraintDistance.h
//
#pragma once
#include "ConstraintBase.h"

/*
================================
ConstraintDistance
================================
*/
class ConstraintDistance : public Constraint {
public:
	ConstraintDistance() : Constraint(),
		m_Jacobian(12 )
	{
		m_cachedLambda = 0.0f;
		m_baumgarte = 0.0f;
	}
	
	virtual void SetBodies(Body* bodyA, Vec3 anchorA, Vec3 axisA, Body* bodyB, Vec3 anchorB, Vec3 axisB) override;

	void PreSolve( const float dt_sec ) override;
	void Solve() override;
	void PostSolve() override;

private:
	// MatMN m_Jacobian;
	VecN m_Jacobian;

	// VecN m_cachedLambda;
	float m_cachedLambda;
	float m_baumgarte;
	float m_originalDistance;
};