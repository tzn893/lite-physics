//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

#include "Physics/Shapes/ShapeFactory.h"


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

	// 计算冲量
	for (auto& body : m_bodies)
	{
		float mass = body->GetMass();

		Vec3 gravityImpulse = m_gravity * mass * dt_sec;
		body->ApplyImpulse(gravityImpulse);
	}

	for (int i = 0; i < m_bodies.size(); i++)
	{
		for (int j = 0; j < m_bodies.size(); j++)
		{
			if (i == j || 
				(m_bodies[i]->HasInfintyMass() && m_bodies[j]->HasInfintyMass())
			)
				continue;

			contact_t contact;

			if (Intersect(m_bodies[i], m_bodies[j], contact))
			{
				ResolveContact(contact);
			}
		}
	}

	// 计算速度
	for (auto& body : m_bodies)
	{
		body->UpdatePosition(dt_sec);
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
	default:
		break;
	}

	m_bodies->push_back(body);
}
