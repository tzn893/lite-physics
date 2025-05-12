//
//  Body.cpp
//
#include "Body.h"
#include "../Math/Helpers.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
m_position( 0.0f ),
m_orientation( 0.0f, 0.0f, 0.0f, 1.0f ),
m_shape( NULL ){
}

void Body::Initialize(Vec3 pos, Quat ori, float mass, Shape* shape, float elasity, float firction)
{
    m_position = pos;
	m_orientation = ori;
	m_linearVelocity = Vec3(0, 0, 0);
    m_shape = shape;
    
    if (mass >= InfinityMass)
    {
        m_invMass = 0.0f;
    }
    else
    {
        m_invMass = 1.0f / mass;
    }

    m_elasity = Clamp(elasity, 0.0f, 1.0f);
    m_friction = Clamp(firction, 0.0f, 1.0f);

}

float Body::GetMass() const
{
     if(m_invMass != 0) 
         return 1.0f / m_invMass; 
     return InfinityMass;
}

float Body::GetInvMass() const
{
    return m_invMass;
}

bool Body::HasInfintyMass()
{
    return GetMass() >= InfinityMass;
}

void Body::ApplyImpulse(Vec3 impulse)
{
    if (HasInfintyMass())
        return;
	m_linearVelocity += impulse * m_invMass;

    // 限制线速度大小
    if (m_linearVelocity.GetMagnitude() > maxLinearVelocity)
    {
		m_linearVelocity = m_linearVelocity.Dir() * maxLinearVelocity;
    }
}

void Body::ApplyImpulse(Vec3 impulse, Vec3 position)
{
    if (HasInfintyMass())
    {
        return;
    }
    
    Vec3 relativePosition = position - GetCenterOfMassWorldSpace();
    Vec3 torch = relativePosition.Cross(impulse);//impulse.Cross(relativePosition);
    // relativePosition.Cross();

    // 更新线速度
    ApplyImpulse(impulse);
    ApplyTorch(torch);
}

void Body::ApplyTorch(Vec3 torch)
{
    // 更新角速度
    m_angularVelocity += GetInverseInertiaTensorWorldSpace() * torch;

    // 限制角速度大小
    if (m_angularVelocity.GetMagnitude() > maxAngularVelocity)
    {
        m_angularVelocity = m_angularVelocity.Dir() * maxAngularVelocity;
    }
}

Mat3 Body::GetInertiaTensorLocalSpace() const
{
	Mat3 tensor = m_shape->InertiaTensor();
    tensor *= GetMass();

    return tensor;
}

Mat3 Body::GetInverseInertiaTensorLocalSpace() const
{
    Mat3 invTensor = m_shape->InertiaTensor().Inverse();
    
    return invTensor * m_invMass;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace() const
{
    Mat3 invTensor = m_shape->InertiaTensor().Inverse();
    Mat3 oriMat = m_orientation.ToMat3();

	invTensor = oriMat * invTensor * oriMat.Transpose() * m_invMass;

    return invTensor;
}

Mat3 Body::GetInertialTensorWorldSpace() const
{
    Mat3 tensor = m_shape->InertiaTensor();
    Mat3 oriMat = m_orientation.ToMat3();

    tensor = oriMat * tensor * oriMat.Transpose() * m_invMass;

    return tensor;
}

Mat3 Body::GetInertialTensorLocalSpace() const
{
    return m_shape->InertiaTensor();
}

Vec3 Body::GetCenterOfMassWorldSpace() const
{
	Vec3 centerOfMass = m_shape->GetCenterOfMass();

    return centerOfMass + m_position;
}

Vec3 Body::GetCenterOfMassLocalSpace() const
{
    return m_shape->GetCenterOfMass();
}

Vec3 Body::WorldSpacePointToLocalSpace(const Vec3& pt) const
{
    Quat invOri = m_orientation.Inverse();
    Vec3 relativePositionWorld = GetCenterOfMassWorldSpace() - pt;

	Vec3 relativePositionLocal = invOri.RotatePoint(relativePositionWorld);
	return relativePositionLocal;
}

Vec3 Body::LocalSpacePointToWorldSpace(const Vec3& pt) const
{
	Vec3 rotatedPt = m_orientation.RotatePoint(pt);

    return GetCenterOfMassWorldSpace() + rotatedPt;
}

Vec3 Body::GetBodyPositionWorldSpace()
{
    return m_position;
}

void Body::UpdatePosition(float dt)
{
    if (HasInfintyMass())
    {
        return;
    }

    m_position = m_linearVelocity * dt + m_position;
    // 更新角速度
	// 对于非轴对称物体，其质心不等于中心点，此时物体旋转是会受到内部力矩的影响
    // 该过程满足欧拉旋转定理： t = Ia + wxIw
    // 在更新角速度过程中已计算 Ia 项
    // 因此在这里需要计算修正项 t = wxIw
	Vec3 accelAngular = GetInverseInertiaTensorWorldSpace() * 
        m_angularVelocity.Cross( GetInertialTensorWorldSpace() * m_angularVelocity);

	m_angularVelocity += accelAngular * dt;

    Vec3 dangle = m_angularVelocity * dt;
    Quat dquat = Quat(dangle, dangle.GetMagnitude());
    // 更新角度
    m_orientation = dquat * m_orientation;
    m_orientation.Normalize();
    
	// 绕着质心旋转的中心点
    Vec3 posCM = GetCenterOfMassWorldSpace();
    Vec3 relativePosition = m_position - posCM;
    m_position = posCM + dquat.RotatePoint(relativePosition);
}

float Body::GetElasity() const
{
	return m_elasity;
}

float Body::GetFriction() const
{
    return m_friction;
}

Shape* Body::GetShape() const
{
    return m_shape;
}

Vec3 Body::GetLinearVelocity()
{
    return m_linearVelocity;
}

Vec3 Body::GetAngularVelocity()
{
    return m_angularVelocity;
}

Quat Body::GetOrientation() const
{
    return m_orientation;
}

void Body::FixPosition(Vec3 position)
{
    m_position += position;
}

Body::~Body()
{
    if (m_shape)
    {
        delete m_shape;
    }
}


