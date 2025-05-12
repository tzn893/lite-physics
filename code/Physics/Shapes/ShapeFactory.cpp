#include "ShapeFactory.h"

ShapeFactory::ShapeFactory()
{
}

Shape* ShapeFactory::CreateShape(const ShapeFactoryDesc& desc)
{
	switch (desc.shapeType)
	{
	case EShape::SHAPE_SPHERE:
		return new ShapeSphere(desc.sphere.radius);
		break;
	case EShape::SHAPE_PLANE:
		return new ShapePlane(desc.plane.width, desc.plane.height);
		break;
	}
    return nullptr;
}

ShapeFactoryDesc ShapeFactoryDescHelper::MakeSphere(float radius)
{
	ShapeFactoryDesc desc;
	desc.sphere.radius = radius;
	desc.shapeType = EShape::SHAPE_SPHERE;

	return desc;
}

ShapeFactoryDesc ShapeFactoryDescHelper::MakePlane(float width, float height)
{
	ShapeFactoryDesc desc;
	desc.plane.height = height;
	desc.plane.width = width;
	desc.shapeType = EShape::SHAPE_PLANE;

	return desc;
}
