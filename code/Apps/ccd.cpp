//
//  main.cpp
//
#include "Renderer/application.h"
#include "Physics/Scene.h"

class CCDApplication : public Application
{
public:

	virtual void BuildScene(SceneBuilder* builder) override
	{

		builder->AddSphere(
			Vec3(0, 0, 0.f), Quat(0, 0, 0, 1),
			1.0f, 1.0f, 1.0f, 1.0f
		);
		builder->AddSphere(
			Vec3(20, 0, 0.f), Quat(0, 0, 0, 1),
			5.0f, 0.2f, 1.0f, 1.0f
		);

		builder->AddPlane(
			Vec3(0, 0, -1), Quat(0, 0, 0, 1),
			Body::InfinityMass, 0.8f, 100.0f, 100.0f, 
			1.0f
		);

	}

	virtual void UpdateScene(float dt_sec) override
	{
		if (firstFrame)
		{
			firstFrame = false;
			m_scene->m_bodies[0]->ApplyImpulse(Vec3(1000, 0, 0));
		}

		if (m_inputBuffer.GetKeyHold(IK_A))
		{
			Body* a = m_scene->m_bodies[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();

			position += Vec3(0, 0, 0.5);
			Vec3 impulse = Vec3(0, 10, 0) * dt_sec;
			a->ApplyImpulse(impulse, position);
		}
		if (m_inputBuffer.GetKeyHold(IK_D))
		{
			Body* a = m_scene->m_bodies[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();
			Vec3 torch = Vec3(10, 0, 0) * dt_sec;
			a->ApplyTorch(torch);
		}
		if (m_inputBuffer.GetKeyHold(IK_W))
		{
			Body* a = m_scene->m_bodies[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();
			Vec3 torch = Vec3(0, 10, 0) * dt_sec;
			a->ApplyTorch(torch);
		}
		if (m_inputBuffer.GetKeyHold(IK_S))
		{
			Body* a = m_scene->m_bodies[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();
			Vec3 torch = Vec3(0, -10, 0) * dt_sec;
			a->ApplyTorch(torch);
		}
	}

	void Start()
	{
		firstFrame = true;
		// 取消掉这段注释以开启CCD
		m_scene->m_bodies[0]->SetEnableCCD(true);
	}

	bool firstFrame = false;
};



DEFINE_APPLICATION_ENTRANCE(CCDApplication)
