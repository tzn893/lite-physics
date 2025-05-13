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
	

	void AddSphere(Vec3 position, Quat orientation, float mass, 
		float elasity, float radius, float friction);

	void AddPlane(Vec3 position, Quat orientation, float mass,
		float elasity, float width, float height, float friction);

	void Reset();

	void Clear();

private:

	struct Command
	{
		EShape type;
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
	};

	void ExecuteCommand(const Command& command);

	std::vector<Command>  m_commands;
	std::vector<Body*>*   m_bodies;
	ShapeFactory	 	  m_shapeFactory;
};


class Scene {
	friend class SceneBuilder;
public:
	Scene() { m_bodies.reserve(128);  m_builder = nullptr; }
	~Scene();

	void Reset();
	SceneBuilder* BuildScene();
	void Update( const float dt_sec );	

	std::vector< Body* > m_bodies;
	std::vector< Constraint * >	m_constraints;
	ManifoldCollector m_manifolds;

	SceneBuilder* m_builder;

	Vec3 m_gravity = Vec3(0, 0, -10.0f);
	std::vector<contact_t> m_contactBuffer;
};

