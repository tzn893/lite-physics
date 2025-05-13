//
//	Contact.h
//
#pragma once
#include "Body.h"


struct contact_t {
	Vec3 ptOnA_WorldSpace; // ������ռ�����ϵ��A�ĽӴ���
	Vec3 ptOnB_WorldSpace; // ������ռ�����ϵ��B�ĽӴ���
	Vec3 ptOnA_LocalSpace; // �ھֲ��ռ�����ϵ��A�ĽӴ���
	Vec3 ptOnB_LocalSpace; // �ھֲ��ռ�����ϵ��B�ĽӴ���

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