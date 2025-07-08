//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

#include "Physics/Shapes/ShapeFactory.h"
#include <algorithm>

Vec3 g_global_gravity = Vec3(0, 0, -10.0f);

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

	// printf("body gravity\n");

	// 计算冲量
	for (auto& body : m_bodies)
	{
		if (!body->GravityEnabled())
		{
			continue;
		}
		float mass = body->GetMass();

		Vec3 gravityImpulse = g_global_gravity * mass * dt_sec;
		body->ApplyImpulse(gravityImpulse);
	}

	// printf("board phase\n");

	// 碰撞检测 broadPhase
	std::vector<collisionPair_t> pairs;
	BroadPhase(m_bodies, pairs, dt_sec);

	struct PositionContactInfo
	{
		Body* a, * b;
		Vec3 bodyA_WorldPos;
		Vec3 bodyB_WorldPos;
		Vec3 normal;
	};
	std::vector<PositionContactInfo> positionContactInfos;

	// 碰撞检测 narrowPhase
	for (int i = 0;i < pairs.size();i++)
	{
		Body* bodyI = m_bodies[pairs[i].a];
		Body* bodyJ = m_bodies[pairs[i].b];

		if ((bodyI->HasInfintyMass() && bodyJ->HasInfintyMass()))
			continue;

		IntersectionManifold manifold;

		

		if (Intersect(bodyI, bodyJ, dt_sec, manifold))
		{
			contact_t deepestContact = manifold.contacts[0];

			for (int i = 0; i < manifold.contactCount; i++)
			{
				m_contactBuffer[frameContactCount++] = manifold.contacts[i];// ResolveContact(contact);
				if (deepestContact.separationDistance < manifold.contacts[i].separationDistance)
				{
					deepestContact = manifold.contacts[i];
				}
			}

			positionContactInfos.push_back(PositionContactInfo{ deepestContact.bodyA, deepestContact.bodyB, deepestContact.ptOnA_WorldSpace, deepestContact.ptOnB_WorldSpace, deepestContact.normal });
		}
	}

	/*
	// 对于toi等于0的接触点，生成对应的penertration constraint
	int framePenertrationConstaints = 0;
	// 对于toi大于0的接触点，解算CCD
	int frameCCDContactCount = 0;

	for (int i = 0;i < frameContactCount; i++)
	{
		if (m_contactBuffer[i].timeOfImpact == 0.0f)
		{
			// TODO 更好的分配策略
			ConstraintPenetration* constraint = new ConstraintPenetration();
			constraint->SetBodies(m_contactBuffer[i].bodyA,
				m_contactBuffer[i].ptOnA_LocalSpace,
				m_contactBuffer[i].normal,
				m_contactBuffer[i].bodyB,
				m_contactBuffer[i].ptOnB_LocalSpace,
				m_contactBuffer[i].normal);

			constraint->SetNormal(m_contactBuffer[i].normal * -1.0f);

			float friction = m_contactBuffer[i].bodyA->GetFriction()* m_contactBuffer[i].bodyB->GetFriction();

			constraint->SetFriction(friction);
			m_constraints.push_back(constraint);
			framePenertrationConstaints++;
		}
		else
		{
			std::swap(m_contactBuffer[frameCCDContactCount++], m_contactBuffer[i]);
		}
	}
	*/


	// printf("contact sorting\n");

	// 按 time of impact对所有contact排序
	std::sort(m_contactBuffer.begin(), m_contactBuffer.begin() + frameContactCount,
		[](const contact_t& a, const contact_t& b) 
		{
			if (a.timeOfImpact == b.timeOfImpact)
			{
				return a.separationDistance < b.separationDistance;
			}

			return a.timeOfImpact < b.timeOfImpact;
		}
	);


	// printf("ccd solving\n");
	// 按时间顺序依次处理接触点
	float accumlatedTime = 0.0;
	// 按time of impact处理contact
	for (int i = 0;i <frameContactCount;i++)
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


	
	for (int i = 0;i < positionContactInfos.size();i++)
	{
		PositionContactInfo& contact = positionContactInfos[i];
		Body* bodyA = contact.a;
		Body* bodyB = contact.b;

		Vec3 dist = contact.normal * (contact.bodyB_WorldPos - contact.bodyA_WorldPos).Dot(contact.normal);

		bodyA->FixPosition(dist*  bodyA->GetInvMass() / (bodyA->GetInvMass() + bodyB->GetInvMass()));
		bodyB->FixPosition(dist * -bodyB->GetInvMass() / (bodyA->GetInvMass() + bodyB->GetInvMass()));
	}


	// printf("compute constraints\n");
	// 计算约束
	for (auto& constraint : m_constraints)
	{
		constraint->PreSolve(dt_sec);
	}
	for (int iter = 0; iter < 10; iter++)
	{
		for (auto& constraint : m_constraints)
		{
			constraint->Solve();
		}
	}
	for (auto& constraint : m_constraints)
	{
		constraint->PostSolve();
	}

	/*
	printf("clear constraints\n");
	// 清理掉临时的约束
	for (int i = m_constraints.size() - framePenertrationConstaints;i < m_constraints.size();i++)
	{
		delete m_constraints[i];
	}
	m_constraints.erase(m_constraints.end() - framePenertrationConstaints, m_constraints.end());
	*/
 	float remainingTime = dt_sec - accumlatedTime;
	// 计算速度
	for (auto& body : m_bodies)
	{
		body->UpdatePosition(remainingTime);
	}
}

void Scene::UpdateXPBD(const float dt_sec)
{


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

int SceneBuilder::AddSphere(Vec3 position, Quat orientation, float mass, float elasity, float radius, float friction)
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

	return m_bodies->size() - 1;
}

int SceneBuilder::AddPlane(Vec3 position, Quat orientation, float mass, float elasity, float width, float height, float friction)
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

	return m_bodies->size() - 1;
}



int SceneBuilder::AddBox(Vec3 position, Quat orientation, float mass,
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

	return m_bodies->size() - 1;
}

int SceneBuilder::AddConvex(Vec3 position, Quat orientation, float mass,
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

	ExecuteCommand(command);

	return m_bodies->size() - 1;
}

int SceneBuilder::AddDistanceConstrain(int bodyIndexA, Vec3 anchorA, int bodyIndexB, Vec3 anchorB)
{
	Command command;
	command.bodyIndexA = bodyIndexA;
	command.bodyIndexB = bodyIndexB;
	command.axisA = Vec3(1, 0, 0);
	command.axisB = Vec3(1, 0, 0);
	command.anchorA = anchorA;
	command.anchorB = anchorB;
	command.type = Command::ECommandType::DistanceConstraint;

	m_commands.push_back(command);

	ExecuteCommand(command);

	return m_constraints->size() - 1;
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
			(*m_bodies)[command.bodyIndexA], command.anchorA, command.axisA,
			(*m_bodies)[command.bodyIndexB], command.anchorB, command.axisB
		);

		m_constraints->push_back(DistanceConstraint);
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
