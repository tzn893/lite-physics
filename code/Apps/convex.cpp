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
		// TODO 堆叠测试，不同尺度下的GJK
		builder->AddBox(
			Vec3(21.9f, 1.9f, 2.1f), Quat(0.0, 0, 0.0, 1),
			5.0f, 0.8f, Vec3(2.0f, 2.0f, 2.0f), 1.0f
		);

		builder->AddBox(
			Vec3(21.9f, 7.9f, 2.1f), Quat(0.0, 0, 0.0, 1),
			10.0f, 1.0f, Vec3(2.0f, 2.0f, 2.0f), 1.0f
		);

		
		builder->AddBox(
			Vec3(20, 0, -6.0f), Quat(0, 0, 0, 1),
			Body::InfinityMass, 0.3f, Vec3(10.0f, 10.0f, 10.0f), 1.0f
		);
		
	}

	virtual void UpdateScene(float dt_sec) override
	{
		if (m_inputBuffer.GetKeyHold(IK_A))
		{
			Body* a = m_scene->m_bodies[0];

			/*
			Vec3 torch = Vec3(-10, 0, 0) * dt_sec;
			a->ApplyTorch(torch);
			*/
			a->ApplyImpulse(Vec3( 0, 10, 0) * dt_sec);
		}
		if (m_inputBuffer.GetKeyHold(IK_D))
		{
			Body* a = m_scene->m_bodies[0];
			/*
			Vec3 torch = Vec3(10, 0, 0) * dt_sec;
			a->ApplyTorch(torch);
			*/
			a->ApplyImpulse(Vec3( 0,-10, 0) * dt_sec);
		}
		if (m_inputBuffer.GetKeyHold(IK_W))
		{
			Body* a = m_scene->m_bodies[0];
			/*
			Vec3 torch = Vec3(0, 10, 0) * dt_sec;
			a->ApplyTorch(torch);
			*/
			a->ApplyImpulse(Vec3( 0, 0, 10) * dt_sec);
		}
		if (m_inputBuffer.GetKeyHold(IK_S))
		{
			Body* a = m_scene->m_bodies[0];
			/*
			Vec3 torch = Vec3(0, -10, 0) * dt_sec;
			a->ApplyTorch(torch);
			*/
			a->ApplyImpulse(Vec3(0, 0,-10) * dt_sec);
		}
	}

	void Start()
	{
		m_scene->m_bodies[0]->SetEnableGravity(false);
		m_scene->m_bodies[1]->SetEnableGravity(false);
	}

};



DEFINE_APPLICATION_ENTRANCE(CCDApplication)
