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


/*
bool ConvexContainsOrigin(std::vector<ConvexTriangle> triangles, std::vector<MkDifferencePoint> pts)
{
	for (int i = 0;i < triangles.size(); i++)
	{
		Vec3 a = pts[triangles[i].a].pt, b = pts[triangles[i].b].pt, c = pts[triangles[i].c].pt;

		if (DistanceFromTriangle(a, b, c, Vec3(0, 0, 0)) > 0.0f)
		{
			return false;
		}
	}
	return true;
}
*/

static void BuildClippingPlanes(const std::vector<Vec3>& planePts, const Vec3& planeNormal, std::vector<Vec4>& clippingPlanes)
{
	float planeDistance = planePts[0].Dot(planeNormal);

	clippingPlanes.push_back(Vec4(planeNormal.x, planeNormal.y, planeNormal.z, planeDistance));

	for(size_t idx = 0; idx < planePts.size(); idx++)
	{
		Vec3 start = planePts[idx];
		Vec3 end = planePts[(idx + 1) % planePts.size()];
		Vec3 extra = planePts[(idx + 2) % planePts.size()];

		Vec3 clippingPlaneNormal = planeNormal.Cross(end - start).Dir();
		if ((extra - end).Dot(clippingPlaneNormal) > 0)
		{
			clippingPlaneNormal *= -1;
		}

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
	float t = (planeDistance - planeNormal.Dot(start)) / planeNormal.Dot(end - start);
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

	outputPolygon = clipPolygon;
}


void ClippingManifold(const Body* bodyA, const Body* bodyB, const Vec3& ptOnA, const Vec3& ptOnB, IntersectionManifold& manifold)
{
	Vec3 normal = (ptOnB - ptOnA).Dir() * -1;

	ShapeConvexBase* shapeA = dynamic_cast<ShapeConvexBase*>(bodyA->GetShape());
	ShapeConvexBase* shapeB = dynamic_cast<ShapeConvexBase*>(bodyB->GetShape()); 

	// 只有凸包才能使用GJK算法计算manifold
	lite_physics_assert(shapeA && shapeB);

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

		lite_physics_assert(CollisionDistanceBetweenSkewLines(edgeA[0], edgeA[1] - edgeA[0], edgeB[0], edgeB[1] - edgeB[0], l1, l2, m, n));
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

//Kevin's implementation of the Gilbert-Johnson-Keerthi intersection algorithm
//and the Expanding Polytope Algorithm
#define GJK_MAX_NUM_ITERATIONS 64

//Triangle case
void update_simplex3(Vec3& a, Vec3& b, Vec3& c, Vec3& d, int& simp_dim, Vec3& search_dir) {
	/* Required winding order:
	//  b
	//  | \
	//  |   \
	//  |    a
	//  |   /
	//  | /
	//  c
	*/
	Vec3 n = (b - a).Cross(c - a); //triangle's normal
	Vec3 AO = a * -1.0f; //direction to origin

	//Determine which feature is closest to origin, make that the new simplex

	simp_dim = 2;
	if ((b - a).Cross(n).Dot(AO) > 0)
	{ //Closest to edge AB
		c = a;
		//simp_dim = 2;
		search_dir = (b - a).Cross(AO).Cross(b - a);
		return;
	}
	if (n.Cross(c - a).Dot(AO) > 0)
	{ //Closest to edge AC
		b = a;
		//simp_dim = 2;
		search_dir = (c - a).Cross(AO).Cross(c - a);
		return;
	}

	simp_dim = 3;
	if (n.Dot(AO) > 0) { //Above triangle
		d = c;
		c = b;
		b = a;
		//simp_dim = 3;
		search_dir = n;
		return;
	}
	//else //Below triangle
	d = b;
	b = a;
	//simp_dim = 3;
	search_dir = n * -1.0f;
	return;
}


//Tetrahedral case
bool update_simplex4(Vec3& a, Vec3& b, Vec3& c, Vec3& d, int& simp_dim, Vec3& search_dir) {
	// a is peak/tip of pyramid, BCD is the base (counterclockwise winding order)
	//We know a priori that origin is above BCD and below a

	//Get normals of three new faces
	Vec3 ABC = (b - a).Cross(c - a);
	Vec3 ACD = (c - a).Cross(d - a);
	Vec3 ADB = (d - a).Cross(b - a);

	Vec3 AO = a * -1.0f; //dir to origin
	simp_dim = 3; //hoisting this just cause

	//Plane-test origin with 3 faces
	/*
	// Note: Kind of primitive approach used here; If origin is in front of a face, just use it as the new simplex.
	// We just go through the faces sequentially and exit at the first one which satisfies dot product. Not sure this
	// is optimal or if edges should be considered as possible simplices? Thinking this through in my head I feel like
	// this method is good enough. Makes no difference for AABBS, should test with more complex colliders.
	*/
	if (ABC.Dot(AO) > 0) { //In front of ABC
		d = c;
		c = b;
		b = a;
		search_dir = ABC;
		return false;
	}
	if (ACD.Dot(AO) > 0) { //In front of ACD
		b = a;
		search_dir = ACD;
		return false;
	}
	if (ADB.Dot(AO) > 0) { //In front of ADB
		c = d;
		d = b;
		b = a;
		search_dir = ADB;
		return false;
	}

	//else inside tetrahedron; enclosed!
	return true;

	//Note: in the case where two of the faces have similar normals,
	//The origin could conceivably be closest to an edge on the tetrahedron
	//Right now I don't think it'll make a difference to limit our new simplices
	//to just one of the faces, maybe test it later.
}


bool gjk(Body* coll1, Body* coll2, Vec3* mtv) {
	Vec3 a, b, c, d; //Simplex: just a set of points (a is always most recently added)
	Vec3 search_dir = coll1->GetBodyPositionWorldSpace() - coll2->GetBodyPositionWorldSpace(); //initial search direction between colliders

	//Get initial point for simplex
	c = coll2->GetSupportWorldSpace(search_dir, 0.0f) - coll1->GetSupportWorldSpace(search_dir * -1.0f, 0.0f);
	search_dir = c * -1.0f; //search in direction of origin

	//Get second point for a line segment simplex
	b = coll2->GetSupportWorldSpace(search_dir, 0.0f) - coll1->GetSupportWorldSpace(search_dir * -1.0f, 0.0f);

	if (b.Dot(search_dir) < 0) { return false; }//we didn't reach the origin, won't enclose it

	search_dir = (c - b).Cross(b * -1.0f).Cross(c - b); //search perpendicular to line segment towards origin
	if (search_dir == Vec3(0, 0, 0)) { //origin is on this line segment
		//Apparently any normal search vector will do?
		search_dir = (c - b).Cross(Vec3(1, 0, 0)); //normal with x-axis
		if (search_dir == Vec3(0, 0, 0)) search_dir = c - b.Cross(Vec3(0, 0, -1)); //normal with z-axis
	}
	int simp_dim = 2; //simplex dimension

	for (int iterations = 0; iterations < GJK_MAX_NUM_ITERATIONS; iterations++)
	{
		a = coll2->GetSupportWorldSpace(search_dir, 0.0f) - coll1->GetSupportWorldSpace(search_dir * -1.0f, 0.0f);
		if (a.Dot(search_dir) < 0) { return false; }//we didn't reach the origin, won't enclose it

		simp_dim++;
		if (simp_dim == 3) {
			update_simplex3(a, b, c, d, simp_dim, search_dir);
		}
		else if (update_simplex4(a, b, c, d, simp_dim, search_dir)) {
			if (mtv) *mtv = EPA(a, b, c, d, coll1, coll2);
			return true;
		}
	}//endfor
	return false;
}




//Expanding Polytope Algorithm
//Find minimum translation vector to resolve collision
#define EPA_TOLERANCE 0.0001
#define EPA_MAX_NUM_FACES 64
#define EPA_MAX_NUM_LOOSE_EDGES 32
#define EPA_MAX_NUM_ITERATIONS 64
Vec3 EPA(Vec3 a, Vec3 b, Vec3 c, Vec3 d, Body* coll1, Body* coll2) {

	struct Face
	{
		int idxs[3];
		Vec3 normal;
		std::vector<Vec3>* pts;

		Face()
		{
			idxs[0] = 0, idxs[1] = 0, idxs[2] = 0;
			pts = nullptr;
		}

		Face(int a, int b, int c, std::vector<Vec3>& pts)
		{
			idxs[0] = a, idxs[1] = b, idxs[2] = c;
			normal = (pts[b] - pts[a]).Cross(pts[c] - pts[a]);
			this->pts = &pts;
		}

		Vec3 operator[](int idx)
		{
			lite_physics_assert(idx < 3 && idx >= 0);
			return (*pts)[idx];
		}
	};

	struct Edge
	{
		int idxs[2];
		std::vector<Vec3>* pts;
		
		Edge(int a, int b, std::vector<Vec3>& pts)
		{
			idxs[0] = a, idxs[1] = b;
			this->pts = &pts;
		}

		Edge() 
		{
			idxs[0] = 0, idxs[1] = 0;
			this->pts = nullptr;
		}

		Vec3 operator[](int idx)
		{
			lite_physics_assert(idx < 2 && idx >= 0);
			return (*pts)[idx];
		}

		bool operator==(const Edge& other) const
		{
			return idxs[0] == other.idxs[0] && idxs[1] == other.idxs[1] && pts == other.pts;
		}
	};

	Face faces[EPA_MAX_NUM_FACES]; //Array of faces, each with 3 verts and a normal
	std::vector<Vec3> pts{a, b, c, d};


	//Init with final simplex from GJK
	/*
	faces[0][0] = a;
	faces[0][1] = b;
	faces[0][2] = c;
	faces[0][3] = (b - a).Cross(c - a).Dir(); //ABC
	faces[1][0] = a;
	faces[1][1] = c;
	faces[1][2] = d;
	faces[1][3] = (c - a).Cross(d - a).Dir(); //ACD
	faces[2][0] = a;
	faces[2][1] = d;
	faces[2][2] = b;
	faces[2][3] = (d - a).Cross(b - a).Dir(); //ADB
	faces[3][0] = b;
	faces[3][1] = d;
	faces[3][2] = c;
	faces[3][3] = (d - b).Cross(c - b).Dir(); //BDC
	*/
	faces[0] = Face(0, 1, 2, pts);
	faces[1] = Face(0, 2, 3, pts);
	faces[2] = Face(0, 3, 1, pts);
	faces[3] = Face(1, 3, 2, pts);

	int num_faces = 4;
	int closest_face;

	for (int iterations = 0; iterations < EPA_MAX_NUM_ITERATIONS; iterations++) {
		//Find face that's closest to origin
		float min_dist = (faces[0][0]).Dot(faces[0].normal);
		closest_face = 0;
		for (int i = 1; i < num_faces; i++) {
			float dist = (faces[i][0]).Dot(faces[i].normal);
			if (dist < min_dist) {
				min_dist = dist;
				closest_face = i;
			}
		}

		//search normal to face that's closest to origin
		Vec3 search_dir = faces[closest_face].normal;
		Vec3 p = coll2->GetSupportWorldSpace(search_dir, 0.0f) - coll1->GetSupportWorldSpace(search_dir * -1.0f, 0.0f);

		if (p.Dot(search_dir) - min_dist < EPA_TOLERANCE) {
			//Convergence (new point is not significantly further from origin)
			return faces[closest_face].normal * p.Dot(search_dir); //dot vertex with normal to resolve collision along normal!
		}

		Edge loose_edges[EPA_MAX_NUM_LOOSE_EDGES]; //keep track of edges we need to fix after removing faces
		int num_loose_edges = 0;

		//Find all triangles that are facing p
		for (int i = 0; i < num_faces; i++)
		{
			if ((faces[i].normal).Dot(p - faces[i][0]) > 0) //triangle i faces p, remove it
			{
				//Add removed triangle's edges to loose edge list.
				//If it's already there, remove it (both triangles it belonged to are gone)
				for (int j = 0; j < 3; j++) //Three edges per face
				{
					Edge current_edge(faces[i].idxs[j], faces[i].idxs[(j + 1) % 3], pts);
					bool found_edge = false;
					for (int k = 0; k < num_loose_edges; k++) //Check if current edge is already in list
					{
						if (loose_edges[k] == current_edge) {
							//Edge is already in the list, remove it
							//THIS ASSUMES EDGE CAN ONLY BE SHARED BY 2 TRIANGLES (which should be true)
							//THIS ALSO ASSUMES SHARED EDGE WILL BE REVERSED IN THE TRIANGLES (which 
							//should be true provided every triangle is wound CCW)
							std::swap(loose_edges[num_loose_edges - 1], loose_edges[k]); //with last edge in list
							// loose_edges.erase(loose_edges.begin() + k);
							num_loose_edges--;
							found_edge = true;
							break; //exit loop because edge can only be shared once
						}
					}//endfor loose_edges

					if (!found_edge) { //add current edge to list
						// assert(num_loose_edges<EPA_MAX_NUM_LOOSE_EDGES);
						if (num_loose_edges >= EPA_MAX_NUM_LOOSE_EDGES) break;
						loose_edges[num_loose_edges] = current_edge;
						num_loose_edges++;
					}
				}

				//Remove triangle i from list
				faces[i][0] = faces[num_faces - 1][0];
				faces[i][1] = faces[num_faces - 1][1];
				faces[i][2] = faces[num_faces - 1][2];
				faces[i][3] = faces[num_faces - 1][3];
				num_faces--;
				i--;
			}//endif p can see triangle i
		}//endfor num_faces

		//Reconstruct polytope with p added
		for (int i = 0; i < num_loose_edges; i++)
		{
			// assert(num_faces<EPA_MAX_NUM_FACES);
			if (num_faces >= EPA_MAX_NUM_FACES) break;
			faces[num_faces][0] = loose_edges[i][0];
			faces[num_faces][1] = loose_edges[i][1];
			faces[num_faces][2] = p;
			faces[num_faces].normal = (loose_edges[i][0] - loose_edges[i][1]).Cross(loose_edges[i][0] - p);
			
			pts.push_back(p);

			//Check for wrong normal to maintain CCW winding
			float bias = 0.000001; //in case dot result is only slightly < 0 (because origin is on face)
			if (faces[num_faces][0].Dot(faces[num_faces].normal) + bias < 0) {
				/*
				Vec3 temp = faces[num_faces][0];
				faces[num_faces][0] = faces[num_faces][1];
				faces[num_faces][1] = temp;
				*/
				std::swap(faces[num_faces].idxs[0], faces[num_faces].idxs[1]);
				faces[num_faces].normal = faces[num_faces].normal * -1.0f;
			}
			num_faces++;
		}
	} //End for iterations
	printf("EPA did not converge\n");
	//Return most recent closest point
	return faces[closest_face][3] * faces[closest_face][0].Dot(faces[closest_face][3]);
}


/*
================================
GJK_DoesIntersect
================================

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

		ConvexTriangle tri{ i, j, k };

		// 三角形法线应当朝外，这意味着另一个点到三角形的距离必须小于0，因此若该距离大于0则需要调整三角形顶点顺序
		if (DistanceFromTriangle(points[tri.a].pt, points[tri.b].pt, points[tri.c].pt,
			Vec3(0, 0, 0)) > 0)
		{
			std::swap(tri.b, tri.c);
		}

		triangles.push_back(tri);
	}


	// debug 用，检查当前构造的凸包是否合理
	lite_physics_assert(ConvexContainsOrigin(triangles, points));

	while (true)
	{
		// 找到当前距离原点最近的三角形，沿着其法线方向扩张
		int closestTriangleIdx = FindClosestTriangle();
		ConvexTriangle closestTriangle = triangles[closestTriangleIdx];

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

		// debug 用，检查当前构造的凸包是否合理
		lite_physics_assert(ConvexContainsOrigin(triangles, points));
	}

}


// 找到距离原点最近的三角形
int EPASolver::FindClosestTriangle()
{
	int minIdx = 0;
	float minDistance = 1e10;

	for (int idx = 0; idx < triangles.size(); idx++)
	{
		ConvexTriangle tri = triangles[idx];

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

float EPASolver::ClosestPointDistanceFromTriangle(const ConvexTriangle& tri, const Vec3& pt)
{
	Vec3 trianglePts[] =
	{ points[tri.a].pt - pt, points[tri.b].pt - pt, points[tri.c].pt - pt};

	Vec3 lambda = SignedVolume(trianglePts[0], trianglePts[1], trianglePts[2]);
	Vec3 closestPt = trianglePts[0] * lambda.x + trianglePts[1] * lambda.y + trianglePts[2] * lambda.z;

	return closestPt.GetMagnitude();
}

float EPASolver::ProjectedSignedDistanceFromTriangle(const ConvexTriangle& tri, const Vec3& pt)
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
		ConvexTriangle tri = triangles[idx];
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
		ConvexTriangle tri{ lastPtIdx, edge.a, edge.b };
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
	lite_physics_assert(simplexCnt >= 2 && simplexCnt <= 4);

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
*/


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
				normal *= -1;
			}

			MkDifferencePoint pt = MkDifferencePoint::Support(bodyA, bodyB, normal * -1, bias);
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
}*/