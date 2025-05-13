//
//	Contact.h
//
#pragma once
#include "Body.h"


struct contact_t {
	Vec3 ptOnA_WorldSpace; // 在世界空间坐标系下A的接触点
	Vec3 ptOnB_WorldSpace; // 在世界空间坐标系下B的接触点
	Vec3 ptOnA_LocalSpace; // 在局部空间坐标系下A的接触点
	Vec3 ptOnB_LocalSpace; // 在局部空间坐标系下B的接触点

	Vec3 normal;	// In World Space coordinates
	float separationDistance = 1e-6f;	// positive when non-penetrating, negative when penetrating
	float timeOfImpact = 0;

	Body * bodyA = nullptr;
	Body * bodyB = nullptr;

	bool HasIntersection()
	{
		return separationDistance < 0.0f || timeOfImpact > 0;
	}

};

void ResolveContact( contact_t & contact );