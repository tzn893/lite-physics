#include "SAT.h"
#include "Math/Helpers.h"

SATSolver::SATSolver(Body* bodyA, Body* bodyB)
{
	m_initializedSuccessfully = true;

	CollectBodyGeomtry(bodyA, m_possibleAxis);
	int bodyAAxisCnt = m_possibleAxis.size();
	CollectBodyGeomtry(bodyB, m_possibleAxis);
	int bodyBAxisCnt = m_possibleAxis.size() - bodyAAxisCnt;

	m_bodyA = bodyA;
	m_bodyB = bodyB;
	
	// 为了防止边边相交的情况，对每两个轴形成的法线方向做分离测试
	for (int i = 0;i < bodyAAxisCnt; i++)
	{
		for (int j = 0;j < bodyBAxisCnt; j++)
		{
			Vec3 axisA = m_possibleAxis[i];
			Vec3 axisB = m_possibleAxis[j + bodyAAxisCnt];

			Vec3 NewAxis = (axisA.Cross(axisB));
			if (NewAxis.GetLengthSqr() >= 1e-8f)
			{
				m_possibleAxis.push_back(NewAxis.Dir());
			}
		}
	}

}

bool SATSolver::HasIntersection(Vec3& closestPtA, Vec3& closestPtB, float& depth, Vec3& normal)
{
	assert(false);
	assert(m_initializedSuccessfully);

	depth = -1e10;

	// 检查所有可能的边，找到对应的最小距离
	for (auto& axis : m_possibleAxis) 
	{
		auto [pA, pB, hasIntersection] = CheckAxis(axis);
		
		float axisProjedDistance = axis.Dot(pA - pB);
		if ((hasIntersection && axisProjedDistance >= 0) || 
			(!hasIntersection && axisProjedDistance < 0))
		{
			axis = axis * -1;
			axisProjedDistance *= -1;
		}

		if (axisProjedDistance >= depth)
		{
			depth = axisProjedDistance;
			
			closestPtA = pA;
			closestPtB = pB;
			normal = axis;
		}

		// 找到了分离轴，说明两物体不可能相交，直接返回
		if (!hasIntersection)
		{
			normal = normal;
			return false;
		}
	}
	
	return true;
}


void SATSolver::CollectBodyGeomtry(Body* body,  std::vector<Vec3>& outAxis)
{
	if (!body->CollectSeperateAxis(outAxis))
	{
		m_initializedSuccessfully = false;
	}
}

std::tuple<Vec3, Vec3, bool> SATSolver::CheckAxis(const Vec3& axis)
{
	Vec3 minA = m_bodyA->GetSupportWorldSpace(axis * -1, 0);
	Vec3 maxA = m_bodyA->GetSupportWorldSpace(axis, 0);

	Vec3 minB = m_bodyB->GetSupportWorldSpace(axis * -1, 0);
	Vec3 maxB = m_bodyB->GetSupportWorldSpace(axis, 0);

	float projMinA = axis.Dot(minA);
	float projMinB = axis.Dot(minB);
	float projMaxA = axis.Dot(maxA);
	float projMaxB = axis.Dot(maxB);

	if (projMinA <= projMinB && projMaxA >= projMinB)
	{
		return std::make_tuple(maxA, minB, true);
	}

	if (projMinB <= projMinA && projMaxB >= projMinA)
	{
		return std::make_tuple(minA, maxB, true);
	}

	if (projMinB > projMaxA)
	{
		return std::make_tuple(maxA, minB, false);
	}

	if (projMinA > projMaxB)
	{
		return std::make_tuple(minA, maxB, false);
	}

	return std::tuple<Vec3, Vec3,  bool>();
}
