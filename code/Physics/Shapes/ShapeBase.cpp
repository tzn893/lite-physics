#include "ShapeBase.h"

PointArrayAccessor::PointArrayAccessor(const std::vector<Vec3>& data):
	data(data.data()), count(data.size())
{

}

Vec3 PointArrayAccessor::operator[](size_t idx) const
{
	assert(idx < count);
	return data[idx];
}

int Shape::GetSeperateAxis(std::vector<Vec3>& outAxis)
{
	return -1;
}
