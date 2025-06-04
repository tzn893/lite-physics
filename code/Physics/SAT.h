#pragma once

#include "../Math/Vector.h"
#include "../Math/Quat.h"
#include "../Math/Matrix.h"
#include "../Math/Bounds.h"

#include "Body.h"
#include "Shapes.h"
#include <vector>

/* TODO ’“µΩΩªµ„ */
class SATSolver
{
public:

	SATSolver(Body* bodyA, Body* bodyB);

	bool HasIntersection(Vec3& closestPtA, Vec3& closestPtB, float& depth, Vec3& normal);

private:

	void CollectBodyGeomtry(Body* body, std::vector<Vec3>& outNormals);

	std::tuple<Vec3 , Vec3, bool> CheckAxis(const Vec3& axis);

	std::vector<Vec3> m_possibleAxis;
	bool m_initializedSuccessfully;

	Body* m_bodyA, * m_bodyB;
};