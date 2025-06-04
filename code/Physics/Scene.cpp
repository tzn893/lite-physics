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

	/*
	for (auto& constraint : m_constraints)
	{
		constraint->PreSolve(dt_sec);
	}
	for (auto& constraint : m_constraints)
	{
		constraint->Solve();
	}
	*/

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
	m_constraints = &scene->m_constraints;
}

void SceneBuilder::AddSphere(Vec3 position, Quat orientation, float mass, float elasity, float radius, float friction)
{
	Command command;
	command.position = position;
	command.orientation = orientation;
	command.mass = mass;
	command.elasity = elasity;
	command.type = Command::ECommandType::Sphere;
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
	command.type = Command::ECommandType::Plane;
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
	command.type = Command::ECommandType::Box;

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
	command.type = Command::ECommandType::Convex;

	command.convex.pts = std::vector<Vec3>(pts, pts + numPt);

	m_commands.push_back(command);
}

void SceneBuilder::AddDistanceConstrain(Body* bodyA, Vec3 anchorA, Vec3 axisA,
	Body* bodyB, Vec3 anchorB, Vec3 axisB)
{
	Command command;
	command.bodyA = bodyA;
	command.bodyB = bodyB;
	command.axisA = axisA;
	command.axisB = axisB;
	command.anchorA = anchorA;
	command.anchorB = anchorB;
	command.type = Command::ECommandType::DistanceConstraint;

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
	for (int i = 0;i < m_constraints->size();i++)
	{
		delete (*m_constraints)[i];
	}
	m_constraints->clear();
}

void SceneBuilder::ExecuteCommand(const Command& command)
{
	switch (command.type)
	{
	case Command::ECommandType::Sphere:
	{
		Body* body = new Body();
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

		m_bodies->push_back(body);
		break;
	}
		
	case Command::ECommandType::Plane:
	{
		Body* body = new Body();
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
		
		m_bodies->push_back(body);
		break;
	}
	case Command::ECommandType::Box:
	{
		Body* body = new Body();
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

		m_bodies->push_back(body);
		break;
	}
	case Command::ECommandType::Convex:
	{
		Body* body = new Body();
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

		m_bodies->push_back(body);
		break;
	}
	case Command::ECommandType::DistanceConstraint:
	{
		ConstraintDistance* DistanceConstraint = new ConstraintDistance();

		DistanceConstraint->SetBodies
		(
			command.bodyA, command.anchorA, command.axisA,
			command.bodyB, command.anchorB, command.axisB
		);
		break;
	}
	default:
		break;
	}

}

const std::vector<Body*>& Scene::GetBodies() const
{
	// TODO: 在此处插入 return 语句
	return m_bodies;
}
