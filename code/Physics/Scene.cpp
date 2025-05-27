//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

#include "Physics/Shapes/ShapeFactory.h"
#include <algorithm>


Scene::~Scene() 
{
	if (m_builder != nullptr)
	{
		m_builder->Clear();
		delete m_builder;
	}
}


void Scene::Reset() {
	if (m_builder != nullptr)
	{
		m_builder->Reset();
	}

	m_contactBuffer.clear();
	m_contactBuffer.resize(m_bodies.size() * m_bodies.size());
}

SceneBuilder* Scene::BuildScene()
{
	if (m_builder == nullptr)
	{
		m_builder = new SceneBuilder(this);
	}

	return m_builder;
}

void Scene::Update(const float dt_sec) {
	// TODO: Add code
	int frameContactCount = 0;

	// 计算冲量
	for (auto& body : m_bodies)
	{
		if (!body->GravityEnabled())
		{
			continue;
		}
		float mass = body->GetMass();

		Vec3 gravityImpulse = m_gravity * mass * dt_sec;
		body->ApplyImpulse(gravityImpulse);
	}


	// 碰撞检测 broadPhase
	std::vector<collisionPair_t> pairs;
	BroadPhase(m_bodies, pairs, dt_sec);

	// 碰撞检测 narrowPhase
	for (int i = 0;i < pairs.size();i++)
	{
		Body* bodyI = m_bodies[pairs[i].a];
		Body* bodyJ = m_bodies[pairs[i].b];

		if ((bodyI->HasInfintyMass() && bodyJ->HasInfintyMass()))
			continue;

		contact_t contact;

		if (Intersect(bodyI, bodyJ, dt_sec, contact))
		{
			m_contactBuffer[frameContactCount++] = contact;// ResolveContact(contact);
		}
	}

	// 按 time of impact对所有contact排序
	std::sort(m_contactBuffer.begin(), m_contactBuffer.begin() + frameContactCount, 
		[](const contact_t& a, const contact_t& b) 
		{
			return a.timeOfImpact < b.timeOfImpact;
		}
	);


	// 按时间顺序依次处理接触点
	float accumlatedTime = 0.0;
	// 按time of impact处理contact
	for (int i = 0;i < frameContactCount;i++)
	{
		contact_t contact = m_contactBuffer[i];
		float dt = contact.timeOfImpact - accumlatedTime;

		if (contact.bodyA->HasInfintyMass() && contact.bodyB->HasInfintyMass())
		{
			continue;
		}

		if (dt != 0)
		{
			for (auto& body : m_bodies)
			{
				body->UpdatePosition(dt);
			}
		}

		ResolveContact(m_contactBuffer[i]);
		accumlatedTime += dt;
	}

	float remainingTime = dt_sec - accumlatedTime;
	// 计算速度
	for (auto& body : m_bodies)
	{
		body->UpdatePosition(remainingTime);
	}
}

SceneState Scene::GetCurrentState()
{
	SceneState state;
	for (auto& body : m_bodies)
	{
		state.bodies.push_back(body);
		state.bodyStates.push_back(body->GetCurrentState());
	}

	return state;
}

void Scene::RestoreState(const SceneState& state)
{
	for (int idx = 0; idx < state.bodyStates.size(); idx++)
	{
		assert(state.bodies[idx] == m_bodies[idx]);
		m_bodies[idx]->RestoreState(state.bodyStates[idx]);
	}
}



SceneBuilder::SceneBuilder(Scene* scene)
{
	m_bodies = &scene->m_bodies;
}

void SceneBuilder::AddSphere(Vec3 position, Quat orientation, float mass, float elasity, float radius, float friction)
{
	Command command;
	command.position = position;
	command.orientation = orientation;
	command.mass = mass;
	command.elasity = elasity;
	command.type = EShape::SHAPE_SPHERE;
	command.sphere.radius = radius;
	command.friction = friction;

	m_commands.push_back(command);

	ExecuteCommand(command);
}

void SceneBuilder::AddPlane(Vec3 position, Quat orientation, float mass, float elasity, float width, float height, float friction)
{
	Command command;
	command.position = position;
	command.orientation = orientation;
	command.mass = mass;
	command.elasity = elasity;
	command.type = EShape::SHAPE_PLANE;
	command.plane.width = width;
	command.plane.height = height;
	command.friction = friction;

	m_commands.push_back(command);

	ExecuteCommand(command);

}



void SceneBuilder::AddBox(Vec3 position, Quat orientation, float mass,
	float elasity, Vec3 extent, float friction)
{
	Command command;
	command.position = position;
	command.orientation = orientation;
	command.elasity = elasity;
	command.mass = mass;
	command.friction = friction;
	command.type = EShape::SHAPE_BOX;

	command.box.extent = extent;

	m_commands.push_back(command);

	ExecuteCommand(command);
}

void SceneBuilder::AddConvex(Vec3 position, Quat orientation, float mass,
	float elasity, const Vec3* pts, int numPt, float friction)
{
	Command command;
	command.position = position;
	command.orientation = orientation;
	command.elasity = elasity;
	command.mass = mass;
	command.friction = friction;
	command.type = EShape::SHAPE_CONVEX;

	command.convex.pts = std::vector<Vec3>(pts, pts + numPt);

	m_commands.push_back(command);
}


void SceneBuilder::Reset()
{
	Clear();

	for (int i = 0; i < m_commands.size(); i++)
	{
		Command& command = m_commands[i];
		ExecuteCommand(command);
	}
}

void SceneBuilder::Clear()
{
	for (int i = 0; i < m_bodies->size(); i++) 
	{
		delete (*m_bodies)[i];
	}
	m_bodies->clear();
}

void SceneBuilder::ExecuteCommand(const Command& command)
{
	Body* body = new Body();

	switch (command.type)
	{
	case EShape::SHAPE_SPHERE:
		body->Initialize(
			command.position,
			command.orientation,
			command.mass,
			m_shapeFactory.CreateShape(
				ShapeFactoryDescHelper::MakeSphere(command.sphere.radius)
			),
			command.elasity,
			command.friction
		);
		break;
	case EShape::SHAPE_PLANE:
		body->Initialize(
			command.position,
			command.orientation,
			command.mass,
			m_shapeFactory.CreateShape(
				ShapeFactoryDescHelper::MakePlane(command.plane.width, command.plane.height)
			),
			command.elasity,
			command.friction
		);
		break;
	case EShape::SHAPE_BOX:
		body->Initialize(
			command.position,
			command.orientation,
			command.mass,
			m_shapeFactory.CreateShape(
				ShapeFactoryDescHelper::MakeBox(command.box.extent.x,
					command.box.extent.y, command.box.extent.z)
			),
			command.elasity,
			command.friction
		);
		break;
	case EShape::SHAPE_CONVEX:
		body->Initialize(
			command.position,
			command.orientation,
			command.mass,
			m_shapeFactory.CreateShape(
				ShapeFactoryDescHelper::MakeConvex(command.convex.pts )
			),
			command.elasity,
			command.friction
		);
		break;
	default:
		break;
	}

	m_bodies->push_back(body);
}
