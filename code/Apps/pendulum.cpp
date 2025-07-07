//
//  main.cpp
//
#include "Renderer/application.h"
#include "Physics/Scene.h"

class PendulumApplication : public Application
{
public:

	virtual void BuildScene(SceneBuilder* builder) override
	{
		int box1Idx = builder->AddBox(
			Vec3(21.9f, 1.9f, 0.0f), Quat(0.0, 0, 0.0, 1),
			1.0f, 0.1f, Vec3(2.0f, 2.0f, 2.0f), 1.0f
		);
		Body* box1Body = m_scene->GetBodies()[box1Idx];

		int box2Idx = builder->AddBox(
			Vec3(21.9f, 1.9f, 4.0f), Quat(0.0, 0, 0.0, 1),
			1.0f, 1.0f, Vec3(2.0f, 2.0f, 2.0f), 1.0f
		);
		Body* box2Body = m_scene->GetBodies()[box2Idx];
		
		builder->AddDistanceConstrain(0, Vec3(0, 0, 0), 1, Vec3(0, 0, 0));


		builder->AddPlane(Vec3(0, 0, -3), Quat(0, 0, 0, 1),
			Body::InfinityMass, 0.1f, 100.0f, 100.0f, 0.5f);
	}

	virtual void UpdateScene(float dt_sec) override
	{
		/*
		if (firstFrame)
		{
			firstFrame = false;
			m_scene->GetBodies()[0]->ApplyImpulse(Vec3(900, 0, 0));
		}

		if (m_inputBuffer.GetKeyHold(IK_A))
		{
			Body* a = m_scene->GetBodies()[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();

			position += Vec3(0, 0, 0.5);
			Vec3 torch = Vec3(-100, 0, 0) * dt_sec;
			a->ApplyTorch(torch);
		}
		if (m_inputBuffer.GetKeyHold(IK_D))
		{
			Body* a = m_scene->GetBodies()[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();
			Vec3 torch = Vec3(100, 0, 0) * dt_sec;
			a->ApplyTorch(torch);
		}
		if (m_inputBuffer.GetKeyHold(IK_W))
		{
			Body* a = m_scene->GetBodies()[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();
			Vec3 torch = Vec3(0, 100, 0) * dt_sec;
			a->ApplyTorch(torch);
		}
		if (m_inputBuffer.GetKeyHold(IK_S))
		{
			Body* a = m_scene->GetBodies()[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();
			Vec3 torch = Vec3(0, -100, 0) * dt_sec;
			a->ApplyTorch(torch);
		}*/

	}

	void Start()
	{
	}

};



DEFINE_APPLICATION_ENTRANCE(PendulumApplication)
