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
		builder->AddPlane(
			Vec3(0, 0, -1), Quat(0, 0, 0, 1),
			Body::InfinityMass, 0.8f, 100.0f, 100.0f, 
			1.0f
		);

	}

	virtual void UpdateScene(float dt_sec) override
	{
		
	}
};



DEFINE_APPLICATION_ENTRANCE(CCDApplication)
