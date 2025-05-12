//
//  main.cpp
//
#include "Renderer/application.h"
#include "Physics/Scene.h"

class BasicApplication : public Application
{
public:

	virtual void BuildScene(SceneBuilder* builder) override
	{

		builder->AddSphere(
			Vec3(0, 0, 0.f), Quat(0, 0, 0, 1),
			1.0f, 1.0f, 1.0f, 1.0f
		);
		builder->AddSphere(
			Vec3(0, 0, -101), Quat(0, 0, 0, 1),
			Body::InfinityMass, 0.8f, 100.0f, 1.0f
		);

	}

	virtual void UpdateScene(float dt_sec) override
	{
		if (m_inputBuffer.GetKeyHold(IK_A))
		{
			Body* a = m_scene->m_bodies[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();

			position += Vec3(0, 0, 0.5);
			Vec3 impulse = Vec3(0, 10, 0) * dt_sec;
			a->ApplyImpulse(impulse, position);
			
			// Vec3 torch = Vec3(-10, 0, 0) * dt_sec;
			// a->ApplyTorch(torch);
		}
		if (m_inputBuffer.GetKeyHold(IK_D))
		{
			Body* a = m_scene->m_bodies[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();
			// position += Vec3(0, 0, 0.5);
			// Vec3 impulse = Vec3(0, -10, 0) * dt_sec;
			Vec3 torch = Vec3(10, 0, 0) * dt_sec;
			a->ApplyTorch(torch);
		}
		if (m_inputBuffer.GetKeyHold(IK_W))
		{
			Body* a = m_scene->m_bodies[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();
			// position += Vec3(0, 0, 0.5);
			//Vec3 impulse = Vec3(10, 0, 0) * dt_sec;
			Vec3 torch = Vec3(0, 10, 0) * dt_sec;
			a->ApplyTorch(torch);
		}
		if (m_inputBuffer.GetKeyHold(IK_S))
		{
			Body* a = m_scene->m_bodies[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();
			// position += Vec3(0, 0, 0.5);
			// Vec3 impulse = Vec3(-10, 0, 0) * dt_sec;
			Vec3 torch = Vec3(0,-10, 0) * dt_sec;
			a->ApplyTorch(torch);
		}
	}
};



DEFINE_APPLICATION_ENTRANCE(BasicApplication)
