//
//	ConstraintBase.h
//
#pragma once
#include "../../Math/Vector.h"
#include "../../Math/Quat.h"
#include "../../Math/Matrix.h"
#include "../../Math/Bounds.h"
#include "../../Math/LCP.h"
#include "../Body.h"
#include <vector>

/*
====================================================
Constraint
====================================================
*/
class Constraint {
public:
	virtual void PreSolve( const float dt_sec ) {}
	virtual void Solve() {}
	virtual void PostSolve() {}

	static Mat4 Left( const Quat & q );
	static Mat4 Right( const Quat & q );

	virtual void SetBodies(Body* bodyA, Vec3 anchorA, Vec3 axisA, Body* bodyB, Vec3 anchorB, Vec3 axisB);

protected:
	MatMN GetInverseMassMatrix() const;
	VecN GetVelocities() const;
	void ApplyImpulses( const VecN & impulses );

public:
	Body * m_bodyA;
	Body * m_bodyB;

	Vec3 m_anchorA;		// The anchor location in world space
	Vec3 m_axisA;		// The axis direction in world space

	Vec3 m_anchorB;		// The anchor location in world space
	Vec3 m_axisB;		// The axis direction in world space
};



