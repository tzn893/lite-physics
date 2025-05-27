//
//	Body.h
//
#pragma once
#include "Math/Vector.h"
#include "Math/Quat.h"
#include "Math/Matrix.h"
#include "Math/Bounds.h"
#include "Shapes.h"

/*
====================================================
Body
====================================================
*/

struct BodyState
{
	Vec3		m_position;
	Quat		m_orientation;
	Vec3		m_linearVelocity;
	Vec3		m_angularVelocity;
};

class Body {
public:

	Body();

	void SetEnableCCD(bool enabled);
	bool CCDEnabled();

	void Initialize(Vec3 pos, Quat ori, float mass, Shape* shape, float elasity, float firction);

	float GetMass() const;
	float GetInvMass() const;

	bool  HasInfintyMass();

	// ֱ�����������ĵĳ���
	void  ApplyImpulse(Vec3 impulse);
	// �����ٶȵĳ���
	void  ApplyImpulse(Vec3 impulse, Vec3 position);
	// ֱ�Ӷ�����ʩ������
	void  ApplyTorch(Vec3 torch);

	Bounds GetBounds();

	Mat3 GetInertiaTensorLocalSpace() const;

	Mat3 GetInverseInertiaTensorLocalSpace() const;
	Mat3 GetInverseInertiaTensorWorldSpace() const;

	Mat3 GetInertialTensorWorldSpace() const;
	Mat3 GetInertialTensorLocalSpace() const;

	Vec3 GetCenterOfMassWorldSpace() const;

	Vec3 GetCenterOfMassLocalSpace() const;

	Vec3 WorldSpacePointToLocalSpace(const Vec3& pt) const;
	Vec3 LocalSpacePointToWorldSpace(const Vec3& pt) const;

	Vec3 GetBodyPositionWorldSpace();

	// ��ȡ����ռ���body��Support
	Vec3 GetSupportWorldSpace(Vec3 Dir, float bias) const;

	void UpdatePosition(float dt);

	float GetElasity() const;
	float GetFriction() const;

	static constexpr float InfinityMass = 1e20f;

	Shape* GetShape() const;

	Vec3 GetLinearVelocity();
	Vec3 GetAngularVelocity();
	
	Quat GetOrientation() const;

	// ��solve contact�Ĺ����ж�Bodyλ����������Ҫ�õ���ĺ����
	void FixPosition(Vec3 offset);

	BodyState GetCurrentState();
	void	  RestoreState(const BodyState& state);

	void SetEnableGravity(bool enableGravity);
	bool GravityEnabled();

	~Body();

private:
	Vec3		m_position;
	Quat		m_orientation;
	Vec3		m_linearVelocity;
	Vec3		m_angularVelocity;
	
	// ��Ҫֱ�ӻ�ȡ������ʹ��GetMass�Լ�HasInfinityMass��ȡ����
	float		m_invMass;

	float		m_elasity;

	float		m_friction;

	Shape *		m_shape;

	static constexpr float maxLinearVelocity = 1e3;
	static constexpr float maxAngularVelocity = 30.0f;

	// �Ƿ���CCD
	bool m_enableCCD;

	bool m_enableGravity;
};