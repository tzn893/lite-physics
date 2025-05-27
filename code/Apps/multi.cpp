//
//  main.cpp
//
#include "Renderer/application.h"
#include "Physics/Scene.h"

class MultiBodyApplication : public Application
{
public:

	virtual void BuildScene(SceneBuilder* builder) override
	{

		builder->AddSphere(
			Vec3(0, 0, 0.f), Quat(0, 0, 0, 1),
			100.0f, 1.0f, 1.0f, 1.0f
		);

		float radius = 0.5;
		int  wcount = 30;
		
		Vec3 Extent = Vec3(radius * wcount * 2, radius * wcount * 2, 0);
		Vec3 center = Vec3(30, 0, radius - 1);

		Vec3 unit = Extent / wcount;
		Vec3 start = center - unit * (wcount / 2.0f);

		for (int x = 0;x < wcount; x++)
		{
			for (int y = 0; y < wcount; y++)
			{
				Vec3 pos = start + unit * Vec3(x, y, 0);

				builder->AddSphere(
					pos, Quat(0, 0, 0, 1),
					1.0f, 0.9f, radius, 1.0f
				);
			}
		}
		
		builder->AddPlane(
			Vec3(50, 0, -1), Quat(0, 0, 0, 1),
			Body::InfinityMass, 0.0f, 200.0f, 200.0f,
			1.0f
		);
	}

	virtual void UpdateScene(float dt_sec) override
	{
		if (firstFrame)
		{
			firstFrame = false;
			m_scene->m_bodies[0]->ApplyImpulse(Vec3(900, 0, 0));
		}

		if (m_inputBuffer.GetKeyHold(IK_A))
		{
			Body* a = m_scene->m_bodies[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();

			position += Vec3(0, 0, 0.5);
			Vec3 torch = Vec3(-100, 0, 0) * dt_sec;
			a->ApplyTorch(torch);
		}
		if (m_inputBuffer.GetKeyHold(IK_D))
		{
			Body* a = m_scene->m_bodies[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();
			Vec3 torch = Vec3(100, 0, 0) * dt_sec;
			a->ApplyTorch(torch);
		}
		if (m_inputBuffer.GetKeyHold(IK_W))
		{
			Body* a = m_scene->m_bodies[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();
			Vec3 torch = Vec3(0, 100, 0) * dt_sec;
			a->ApplyTorch(torch);
		}
		if (m_inputBuffer.GetKeyHold(IK_S))
		{
			Body* a = m_scene->m_bodies[0];
			Vec3 position = a->GetCenterOfMassWorldSpace();
			Vec3 torch = Vec3(0, -100, 0) * dt_sec;
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



DEFINE_APPLICATION_ENTRANCE(MultiBodyApplication)
