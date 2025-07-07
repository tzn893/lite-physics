//
//  Scene.h
//
#pragma once
#include <vector>

#include "Physics/Shapes.h"
#include "Physics/Body.h"
#include "Physics/Constraints.h"
#include "Physics/Manifold.h"
#include "Math/Vector.h"
#include "Physics/Shapes/ShapeFactory.h"

/*
====================================================
Scene
====================================================
*/


class SceneBuilder
{
public:
	SceneBuilder(class Scene* scene);
	

	int AddSphere(Vec3 position, Quat orientation, float mass, 
		float elasity, float radius, float friction);

	int AddPlane(Vec3 position, Quat orientation, float mass,
		float elasity, float width, float height, float friction);

	int AddBox(Vec3 position ,Quat orientation, float mass,
		float elasity, Vec3 extent, float friction);

	int AddConvex(Vec3 position, Quat orientation, float mass,
		float elasity, const Vec3* pts, int numPt, float friction);

	int AddDistanceConstrain(int bodyIndexA, Vec3 anchorA, int bodyIndexB, Vec3 anchorB);

	void Reset();

	void Clear();

private:

	struct Command
	{
		enum ECommandType
		{
			Sphere,
			Plane,
			Box,
			Convex,
			DistanceConstraint
		} type;

		Vec3 position;
		Quat orientation;
		float mass;
		float elasity;
		float friction;

		struct CommandSphereData
		{
			float radius;
		} sphere;

		struct CommandPlaneData
		{
			float width, height;
		} plane;

		struct CommandBoxData
		{
			Vec3 extent;
		} box;

		struct CommandConvex
		{
			std::vector<Vec3> pts;
		} convex;

		int bodyIndexA;
		Vec3 anchorA;
		Vec3 axisA;
		int bodyIndexB;
		Vec3 anchorB;
		Vec3 axisB;
	};

	void ExecuteCommand(const Command& command);

	std::vector<Command>  m_commands;
	std::vector<Body*>*   m_bodies;
	std::vector<Constraint*>* m_constraints;
	ShapeFactory	 	  m_shapeFactory;
};

struct SceneState
{
	std::vector<Body*>	   bodies;
	std::vector<BodyState> bodyStates;
};


class Scene {
	friend class SceneBuilder;
public:
	Scene() { m_bodies.reserve(128);  m_builder = nullptr; }
	~Scene();

	void Reset();
	SceneBuilder* BuildScene();
	void Update( const float dt_sec );

	// 利用XPBD更新场景
	void UpdateXPBD(const float dt_sec);

	SceneState GetCurrentState();
	void RestoreState(const SceneState& state);

	const std::vector<Body*>& GetBodies() const;

private:
	std::vector< Body* > m_bodies;
	std::vector< Constraint *>	m_constraints;

	int maxStepCnt = 0;
	std::vector<SceneState>	m_sceneStates;

	ManifoldCollector m_manifolds;

	SceneBuilder* m_builder;

	
	std::vector<contact_t> m_contactBuffer;

};

