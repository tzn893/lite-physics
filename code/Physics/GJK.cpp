 //
//  GJK.cpp
//
#include "GJK.h"
#include "Intersections.h"
#include "Math/Helpers.h"
#include <tuple>

template<typename T>
using opt = std::optional <T>;

template<typename ...Args>
using tpl = std::tuple<Args...>;

static void BuildClippingPlanes(const std::vector<Vec3>& planePts, const Vec3& planeNormal, std::vector<Vec4>& clippingPlanes)
{
	float planeDistance = planePts[0].Dot(planeNormal);

	clippingPlanes.push_back(Vec4(planeNormal.x, planeNormal.y, planeNormal.z, planeDistance));

	for(size_t idx = 0; idx < planePts.size(); idx++)
	{
		Vec3 start = planePts[idx];
		Vec3 end = planePts[(idx + 1) % planePts.size()];
		Vec3 clippingPlaneNormal = planeNormal.Cross(end - start).Dir();
		float clippingPlaneDistance = clippingPlaneNormal.Dot(start);
		clippingPlanes.push_back(Vec4(clippingPlaneNormal.x, clippingPlaneNormal.y, clippingPlaneNormal.z,
			clippingPlaneDistance));
	}

}

static bool IsVertexInsidePlane(const Vec3& vert, const Vec4& plane)
{
	Vec3 planeNormal = Vec3(plane.x, plane.y, plane.z);
	Vec3 planePt =  planeNormal * plane.w;
	return (vert - planePt).Dot(planeNormal) < 0.0f;
}


static Vec3 FindIntersectionWithPlaneAndPoint(const Vec4& plane, const Vec3& start, const Vec3& end)
{
	Vec3 planeNormal = Vec3(plane.x, plane.y, plane.z);
	float planeDistance = plane.w;
	float t = (planeDistance - planeNormal.Dot(start)) / Max(planeNormal.Dot(end - start), 1e-8f);
	return start * (1 - t) + end * t;
}

// 来自 https://github.com/felipeek/raw-physics/blob/master/src/physics/clipping.cpp
static bool CollisionDistanceBetweenSkewLines(Vec3 p1, Vec3 d1, Vec3 p2, Vec3 d2, Vec3& l1, Vec3& l2, float& _n, float& _m) 
{
	float n1 = d1.x * d2.x + d1.y * d2.y + d1.z * d2.z;
	float n2 = d2.x * d2.x + d2.y * d2.y + d2.z * d2.z;
	float m1 = -d1.x * d1.x - d1.y * d1.y - d1.z * d1.z;
	float m2 = -d2.x * d1.x - d2.y * d1.y - d2.z * d1.z;
	float r1 = -d1.x * p2.x + d1.x * p1.x - d1.y * p2.y + d1.y * p1.y - d1.z * p2.z + d1.z * p1.z;
	float r2 = -d2.x * p2.x + d2.x * p1.x - d2.y * p2.y + d2.y * p1.y - d2.z * p2.z + d2.z * p1.z;

	// Solve 2x2 linear system
	if ((n1 * m2) - (n2 * m1) == 0) {
		return false;
	}
	float n = ((r1 * m2) - (r2 * m1)) / ((n1 * m2) - (n2 * m1));
	float m = ((n1 * r2) - (n2 * r1)) / ((n1 * m2) - (n2 * m1));

	l1 = p1 + d1 * m;
	l2 = p2 + d2 * n;
	_n = n;
	_m = m;

	return true;
}


void SutherlandHodgmanClip(const std::vector<Vec4>& clippingPlanes, std::vector<Vec3> clipPolygon, std::vector<Vec3>& outputPolygon)
{
	outputPolygon = std::vector<Vec3>();
	for (const Vec4& clippingPlane: clippingPlanes)
	{
		for (size_t i = 0; i < clipPolygon.size(); ++i)
		{
			Vec3 startPt = clipPolygon[(i + clipPolygon.size() - 1) % clipPolygon.size()];
			Vec3 endPt = clipPolygon[i];

			bool IsStartInPlane = IsVertexInsidePlane(startPt, clippingPlane);
			bool IsEndInPlane = IsVertexInsidePlane(endPt, clippingPlane);

			if(IsStartInPlane && IsEndInPlane)
			{
				outputPolygon.push_back(endPt);
			}
			else
			{
				Vec3 clippedPt = FindIntersectionWithPlaneAndPoint(clippingPlane, startPt, endPt);
				if(IsStartInPlane && !IsEndInPlane)
				{
					outputPolygon.push_back(clippedPt);
				}
				else if(!IsStartInPlane && IsEndInPlane)
				{
					outputPolygon.push_back(clippedPt);
					outputPolygon.push_back(endPt);
				}
			}
		}

		std::swap(outputPolygon, clipPolygon);
		outputPolygon.clear();
	}
}


void ClippingManifold(const Body* bodyA, const Body* bodyB, const Vec3& ptOnA, const Vec3& ptOnB, IntersectionManifold& manifold)
{
	Vec3 normal = (ptOnB - ptOnA).Dir() * -1;

	ShapeConvexBase* shapeA = dynamic_cast<ShapeConvexBase*>(bodyA->GetShape());
	ShapeConvexBase* shapeB = dynamic_cast<ShapeConvexBase*>(bodyB->GetShape()); 

	// 只有凸包才能使用GJK算法计算manifold
	assert(shapeA && shapeB);

	std::vector<Vec3> edgeA = shapeA->FindClosestEdgeByContact(ptOnA, bodyA->GetCenterOfMassWorldSpace(), bodyA->GetOrientation());
	std::vector<Vec3> edgeB = shapeB->FindClosestEdgeByContact(ptOnB, bodyB->GetCenterOfMassWorldSpace(), bodyB->GetOrientation());

	Vec3 closestFaceANormal, closestFaceBNormal;
	int  closestFaceAIdx, closestFaceBIdx;
	std::vector<Vec3> closestFaceA = shapeA->FindClosestFaceByNormal(normal, bodyA->GetCenterOfMassWorldSpace(), bodyA->GetOrientation(), closestFaceANormal, closestFaceAIdx);
	std::vector<Vec3> closestFaceB = shapeB->FindClosestFaceByNormal(normal * -1, bodyB->GetCenterOfMassWorldSpace(), bodyB->GetOrientation(), closestFaceBNormal, closestFaceBIdx);

	Vec3 edgeNormal = (edgeA[0] - edgeA[1]).Cross(edgeB[0] - edgeB[1]);
	edgeNormal.Normalize();

	float edgeNormalDistance = Abs(edgeNormal.Dot(normal));
	float closestFaceANormalDistance = Abs(closestFaceANormal.Dot(normal));
	float closestFaceBNormalDistance = Abs(closestFaceBNormal.Dot(normal));

	if (edgeNormalDistance > closestFaceANormalDistance && edgeNormalDistance > closestFaceBNormalDistance && Max(closestFaceANormalDistance, closestFaceBNormalDistance) < 0.96)
	{
		Vec3 l1, l2;
		float m, n;

		assert(CollisionDistanceBetweenSkewLines(edgeA[0], edgeA[1] - edgeA[0], edgeB[0], edgeB[1] - edgeB[0], l1, l2, m, n));
		manifold.contactCount = 1;
		contact_t contact;
		contact.bodyA = const_cast<Body*>(bodyA);
		contact.bodyB = const_cast<Body*>(bodyB);
		contact.ptOnA_WorldSpace = l1;
		contact.ptOnB_WorldSpace = l2;
		contact.ptOnA_LocalSpace = bodyA->WorldSpacePointToLocalSpace(l1);
		contact.ptOnB_LocalSpace = bodyB->WorldSpacePointToLocalSpace(l2);
		contact.normal = normal;

		contact.separationDistance = (l2 - l1).Dot(normal);
		contact.timeOfImpact = 0.0f;
		
		manifold.contacts[0] = contact;
	}
	// 使用 sutherland_hodgman 算法计算求交后的 manifold
	else
	{
		bool isFaceAReference = closestFaceANormalDistance > closestFaceBNormalDistance;

		std::vector<Vec4> clippingPlanes;
		std::vector<Vec3> clippedPolygon;
		if(isFaceAReference)
		{
			BuildClippingPlanes(closestFaceA, closestFaceANormal, clippingPlanes);
			SutherlandHodgmanClip(clippingPlanes, closestFaceB, clippedPolygon);
		}
		else
		{
			BuildClippingPlanes(closestFaceB, closestFaceBNormal, clippingPlanes);
			SutherlandHodgmanClip(clippingPlanes, closestFaceA, clippedPolygon);
		}
		
		manifold.contactCount = Min(manifold.maxContactCount, (int)clippedPolygon.size());
		for(int i = 0; i < manifold.contactCount; i++)
		{
			contact_t contact;
			contact.bodyA = const_cast<Body*>(bodyA);
			contact.bodyB = const_cast<Body*>(bodyB);

			if(isFaceAReference)
			{
				// 将裁剪后的多边形顶点投影到多边形表面
				Vec3 clippedPtOnA = closestFaceANormal * (clippingPlanes[0].w - clippedPolygon[i].Dot(closestFaceANormal))
					+ clippedPolygon[i];
				
				contact.ptOnA_WorldSpace = clippedPtOnA;
				contact.ptOnB_WorldSpace = clippedPolygon[i];
			}
			else
			{
				// 将裁剪后的多边形顶点投影到多边形表面
				Vec3 clippedPtOnB = closestFaceBNormal * (clippingPlanes[0].w - clippedPolygon[i].Dot(closestFaceBNormal))
					+ clippedPolygon[i];
				
				contact.ptOnA_WorldSpace = clippedPolygon[i];
				contact.ptOnB_WorldSpace = clippedPtOnB;
			}

			
			contact.separationDistance = (contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace).Dot(normal);
			contact.timeOfImpact = 0.0f;

			contact.ptOnA_LocalSpace = bodyA->WorldSpacePointToLocalSpace(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = bodyB->WorldSpacePointToLocalSpace(contact.ptOnB_WorldSpace);
			
			contact.normal = normal;
			manifold.contacts[i] = contact;
		}
	}


}

/*
================================
GJK_DoesIntersect
================================
*/
void EPASolver::Solve(const Body* bodyA, const Body* bodyB, float bias, MkDifferencePoint* simplexPts
	, Vec3& ptOnA, Vec3& ptOnB)
{
	triangles.clear();
	points = std::vector<MkDifferencePoint>(simplexPts, simplexPts + 4);

	// 计算四面体的中心点
	Vec3 center = Vec3();
	for (int i = 0; i < 4; i++)
	{
		center += points[i].pt;
	}
	center *= 0.25f;


	// 构建四面体对应三角形
	for (int i = 0; i < 4; i++)
	{
		int j = (i + 1) % 4;
		int k = (i + 2) % 4;
		int l = (i + 3) % 4;

		ConvexTriangles tri{ i, j, k };

		// 三角形法线应当朝外，这意味着另一个点到三角形的距离必须小于0，因此若该距离大于0则需要调整三角形顶点顺序
		if (DistanceFromTriangle(points[tri.a].pt, points[tri.b].pt, points[tri.c].pt,
			points[l].pt) > 0)
		{
			std::swap(tri.b, tri.c);
		}

		triangles.push_back(tri);
	}


	while (true)
	{
		// 找到当前距离原点最近的三角形，沿着其法线方向扩张
		int closestTriangleIdx = FindClosestTriangle();
		ConvexTriangles closestTriangle = triangles[closestTriangleIdx];

		Vec3 closestTriangleNormal = TriangleNormal(points[closestTriangle.a].pt,
			points[closestTriangle.b].pt, points[closestTriangle.c].pt);

		MkDifferencePoint newPt = MkDifferencePoint::Support(bodyA, bodyB, closestTriangleNormal, bias);


		// 如果新点仍在凸包内部，说明无法进一步扩张，返回碰撞检测结果
		if (HasPoint(newPt) || ClosestPointDistanceFromTriangle(closestTriangle, newPt.pt) <= 1e-4f)
		{
			// 找到原点投影到最近点上的重心坐标
			MkDifferencePoint closestTrianglePts[3] = { points[closestTriangle.a],
				points[closestTriangle.b], points[closestTriangle.c] };

			Vec3 lambda = SignedVolume(closestTrianglePts[0].pt,
				closestTrianglePts[1].pt, closestTrianglePts[2].pt);

			MkDifferencePoint closestPt = MkDifferencePoint::SimplexInterpolate(closestTrianglePts, 3,
				Vec4(lambda.x, lambda.y, lambda.z, 0));

			ptOnA = closestPt.ptOnA;
			ptOnB = closestPt.ptOnB;

			return;
		}

		// 将新点加入待构建的队列中
		points.push_back(newPt);
		
		// 从凸包中移除面向新点的三角形
		RemovePointFacingTriangle(newPt.pt);
		// 用新点填充新三角形
		FillTrianglesWithNewPoint();
	}

}


// 找到距离原点最近的三角形
int EPASolver::FindClosestTriangle()
{
	int minIdx = 0;
	float minDistance = 1e10;

	for (int idx = 0; idx < triangles.size(); idx++)
	{
		ConvexTriangles tri = triangles[idx];

		// 由于三角形法线方向与原点相反，因此距离一定小于0
		float distance = ClosestPointDistanceFromTriangle(tri, Vec3(0, 0, 0));
		// assert(distance >= 0.0f);
		if (distance < minDistance)
		{
			minDistance = distance;
			minIdx = idx;
		}
	}

	return minIdx;
}

float EPASolver::ClosestPointDistanceFromTriangle(const ConvexTriangles& tri, const Vec3& pt)
{
	Vec3 trianglePts[] =
	{ points[tri.a].pt - pt, points[tri.b].pt - pt, points[tri.c].pt - pt};

	Vec3 lambda = SignedVolume(trianglePts[0], trianglePts[1], trianglePts[2]);
	Vec3 closestPt = trianglePts[0] * lambda.x + trianglePts[1] * lambda.y + trianglePts[2] * lambda.z;

	return closestPt.GetMagnitude();
}

float EPASolver::ProjectedSignedDistanceFromTriangle(const ConvexTriangles& tri, const Vec3& pt)
{
	return DistanceFromTriangle(points[tri.a].pt, points[tri.b].pt, points[tri.c].pt, pt);
}

	// 检查一个新点是否已经在被构造的凸包内部
bool EPASolver::HasPoint(const MkDifferencePoint& pt)
{
	for (int idx = 0; idx < points.size(); idx++)
	{
		if ((pt.pt - points[idx].pt).GetLengthSqr() < 1e-8)
		{
			return true;
		}
	}
	return false;
}

void EPASolver::RemovePointFacingTriangle(const Vec3& pt)
{
	for (int idx = 0; idx < triangles.size();)
	{
		ConvexTriangles tri = triangles[idx];
		// 三角形不应该面向新加入的点
		if (ProjectedSignedDistanceFromTriangle(tri, pt) >= 0)
		{
			triangles.erase(triangles.begin() + idx);
		}
		else
		{
			idx++;
		}
	}
}



void EPASolver::FillTrianglesWithNewPoint()
{
	// 找到三角形中单连通的边
	struct Edge
	{
		int a = 0, b = 0;
		Edge() = default;
		Edge(int a, int b)
		{
			this->a = Min(a, b);
			this->b = Max(a, b);
		}

		bool operator==(const Edge& e)
		{
			return a == e.a && b == e.b;
		}
	};
	
	// 这里碰撞检测的数据量可能比较小，用复杂数据结构加速反而可能负优化
	// 遍历所有三角形的边，如果边不在队列中就加入到队列中，否则将边从队列中移除
	// 由于非孤立边会被两个面共用，因此遍历过后剩余的边为孤立边
	std::vector<Edge> danglingEdges;
	for (int i = 0;i < triangles.size(); i++)
	{
		int triangleIdx[3] = { triangles[i].a, triangles[i].b, triangles[i].c };
		for (int j = 0; j < 3; j++)
		{
			int k = (j + 1) % 3;
			if (auto edgePos = std::find(danglingEdges.begin(), danglingEdges.end(), 
					Edge(triangleIdx[j], triangleIdx[k])); edgePos != danglingEdges.end())
			{
				danglingEdges.erase(edgePos);
			}
			else
			{
				danglingEdges.push_back(Edge(triangleIdx[j], triangleIdx[k]));
			}
		}
	}

	// 从孤立边以及最后一个顶点构造三角形
	for (auto& edge : danglingEdges)
	{
		int lastPtIdx = points.size() - 1;
		ConvexTriangles tri{ lastPtIdx, edge.a, edge.b };
		// 保证三角形的法线总是朝外
		if (ProjectedSignedDistanceFromTriangle(tri, Vec3(0, 0, 0)) > 0)
		{
			std::swap(tri.b, tri.c);
		}
		triangles.push_back(tri);
	}
}




// 支持1, 2, 3类型simplex的Signed Volume函数
// 当原点在simplex内部时，返回{是否包含原点，各个点的权重，投影后点的位置}
tpl<bool, Vec4, Vec3> GeneralSignedVolume(int simplexCnt,const MkDifferencePoint* pts)
{
	assert(simplexCnt >= 2 && simplexCnt <= 4);

	Vec4 lambda;
	bool inSide = false;

	switch (simplexCnt)
	{
	case 2:
		{
			Vec2 lambda2D = SignedVolume(pts[0].pt, pts[1].pt);
			lambda.x = lambda2D.x;
			lambda.y = lambda2D.y;
		}
		break;
	case 3:
		{
			Vec3 lambda3D = SignedVolume(pts[0].pt, pts[1].pt, pts[2].pt);
			lambda.x = lambda3D.x;
			lambda.y = lambda3D.y;
			lambda.z = lambda3D.z;
		}
		break;
	case 4:
		lambda = SignedVolume(pts[0].pt, pts[1].pt, pts[2].pt, pts[3].pt);
		break;
	}

	// 原点到Simplex点最近点位置
	Vec3 projectPt;
	for (int i = 0;i < simplexCnt;i++)
	{
		projectPt += pts[i].pt * lambda[i];
	}
	// 若投影点距离原点足够近，说明两body相交
	inSide = projectPt.GetLengthSqr() < 1e-8f;

	return std::make_tuple(inSide, lambda, projectPt);
}

bool GJK_DoesIntersect( const Body * bodyA, const Body * bodyB ) 
{
	// TODO: Add code
	MkDifferencePoint supportPts[4];

	int simplexCnt = 1;
	supportPts[0] = MkDifferencePoint::Support(bodyA, bodyB, Vec3(1, 1, 1), 0.0f);

	Vec3 searchDir = Vec3(-1, -1, -1).Dir();

	float closestPointFromOriginSqr = supportPts[0].pt.GetLengthSqr();
	MkDifferencePoint closestPoint = supportPts[0];

	bool doseContainOrigin = false;
	bool noProgress = false;

	while (!doseContainOrigin && !noProgress)
	{
		MkDifferencePoint newSupportPt = MkDifferencePoint::Support(bodyA, bodyB, searchDir, 0.0f);

		// 检查新点是否与历史点相同，若相同，证明算法无法取得更大进展，两物体不相交
		for (int i = 0; i < simplexCnt; i++)
		{
			if ((supportPts[i].pt - newSupportPt.pt).GetLengthSqr() < 1e-8f)
			{
				noProgress = true;
				break;
			}
		}
		if (noProgress) break;

		// 将新加入的点放入点队列中，构建新凸包
		supportPts[simplexCnt++] = newSupportPt;

		// 计算新凸包的Signed Volume
		auto [hasIntersection, lambda, projPt] = GeneralSignedVolume(simplexCnt, supportPts);
		doseContainOrigin = hasIntersection;

		// 如果新点无法相比之前无法取得进展，跳出循环结束算法
		if (projPt.GetLengthSqr() >= closestPointFromOriginSqr)
		{
			break;
		}
		closestPointFromOriginSqr = projPt.GetLengthSqr();
		closestPoint = MkDifferencePoint::SimplexInterpolate(supportPts, simplexCnt, lambda);

		// 设置下一个循环更新的方向
		searchDir = (projPt * -1).Dir();

		// 对凸包内点按其权重是否有效排序
		int cIdx = 0, pIdx = 0;
		for (; cIdx < simplexCnt; cIdx++)
		{
			if (lambda[cIdx] != 0)
			{
				std::swap(supportPts[cIdx], supportPts[pIdx++]);
			}
		}
		// 新的凸包点数量为有效点数量
		simplexCnt = pIdx;
	}

	return doseContainOrigin;
}

/*
================================
GJK_ClosestPoints
================================
*/
void GJK_ClosestPoints( const Body * bodyA, const Body * bodyB, Vec3 & ptOnA, Vec3 & ptOnB )
{
	GJK_DoesIntersect(bodyA, bodyB, 2e-4f, ptOnA, ptOnB);
}

/*
================================
GJK_DoesIntersect
================================
*/
bool GJK_DoesIntersect( const Body * bodyA, const Body * bodyB, const float bias, Vec3 & ptOnA, Vec3 & ptOnB ) 
{
	// TODO: Add code
	MkDifferencePoint supportPts[4];

	int simplexCnt = 1;
	supportPts[0] = MkDifferencePoint::Support(bodyA, bodyB, Vec3(1, 1, 1), bias);

	Vec3 searchDir = Vec3(-1, -1, -1).Dir();

	float closestPointFromOriginSqr = supportPts[0].pt.GetLengthSqr();
	MkDifferencePoint closestPoint = supportPts[0];

	bool doseContainOrigin = false;
	bool noProgress = false;

	while (!doseContainOrigin && !noProgress)
	{
		MkDifferencePoint newSupportPt = MkDifferencePoint::Support(bodyA, bodyB, searchDir, bias);

		// 检查新点是否与历史点相同，若相同，证明算法无法取得更大进展，两物体不相交
		for (int i = 0; i < simplexCnt; i++)
		{
			if ((supportPts[i].pt - newSupportPt.pt).GetLengthSqr() < 1e-8f)
			{
				noProgress = true;
				break;
			}
		}

		// 如果新点无法跨过原点，说明不可能存在交点
		if (searchDir.Dot(newSupportPt.pt) < 0.0f)
		{
			noProgress = true;
		}

		if (noProgress) break;

		// 将新加入的点放入点队列中，构建新凸包
		supportPts[simplexCnt++] = newSupportPt;

		// 计算新凸包的Signed Volume
		auto [hasIntersection, lambda, projPt] = GeneralSignedVolume(simplexCnt, supportPts);
		doseContainOrigin = hasIntersection;

		float projPointFromOriginSqr = projPt.GetLengthSqr();
		// 新点没有更大进步，直接退出循环
		if (projPointFromOriginSqr >= closestPointFromOriginSqr)
		{
			noProgress = true;
			break;
		}
		closestPointFromOriginSqr = projPt.GetLengthSqr();
		closestPoint = MkDifferencePoint::SimplexInterpolate(supportPts, simplexCnt, lambda);
		
		// 设置下一个循环更新的方向
		searchDir = (projPt * -1).Dir();

		// 对凸包内点按其权重是否有效排序
		int cIdx = 0, pIdx = 0;
		for (; cIdx < simplexCnt; cIdx++)
		{
			if (lambda[cIdx] != 0)
			{
				std::swap(supportPts[cIdx], supportPts[pIdx++]);
			}
		}
		// 新的凸包点数量为有效点数量
		simplexCnt = pIdx;
	}

	
	// 当两物体相交时，GJK得到的最近不一定是距离最近的点，需要使用EPA算法扩张
	// 需要利用EPA计算交点
	if (doseContainOrigin)
	{
		// 对于退化情况，将simplex补充为四面体
		if (simplexCnt == 1)
		{
			Vec3 Dir = supportPts[0].pt * -1;
			MkDifferencePoint pt = MkDifferencePoint::Support(bodyA, bodyB, Dir, bias);
			supportPts[simplexCnt++] = pt;
		}
		if (simplexCnt == 2)
		{
			Vec3 u, v;
			(supportPts[1].pt - supportPts[0].pt).GetOrtho(u, v);
			supportPts[simplexCnt++] = MkDifferencePoint::Support(bodyA, bodyB, u, bias);
		}
		if (simplexCnt == 3)
		{
			Vec3 normal = TriangleNormal(supportPts[0].pt, supportPts[1].pt, supportPts[2].pt);
			if (normal.Dot(supportPts[0].pt) < 0.0f)
			{
				std::swap(supportPts[0], supportPts[1]);
			}

			MkDifferencePoint pt = MkDifferencePoint::Support(bodyA, bodyB, normal, bias);
			supportPts[simplexCnt++] = pt;
		}

		EPASolver().Solve(bodyA, bodyB, bias, supportPts, ptOnA, ptOnB);
	}
	else
	{
		// 否则，最近点为碰撞检测的结果
		ptOnA = closestPoint.ptOnA;
		ptOnB = closestPoint.ptOnB;
	}

	return doseContainOrigin;
}