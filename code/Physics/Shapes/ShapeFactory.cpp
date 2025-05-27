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
	case EShape::SHAPE_BOX:
		{
			Vec3 pts[8];
			pts[0] = Vec3(desc.box.width / 2.0f, desc.box.length / 2.0f, desc.box.height / 2.0f);
			pts[1] = Vec3(desc.box.width /-2.0f, desc.box.length / 2.0f, desc.box.height / 2.0f);
			pts[2] = Vec3(desc.box.width / 2.0f, desc.box.length /-2.0f, desc.box.height / 2.0f);
			pts[3] = Vec3(desc.box.width /-2.0f, desc.box.length /-2.0f, desc.box.height / 2.0f);
			pts[4] = Vec3(desc.box.width / 2.0f, desc.box.length / 2.0f, desc.box.height /-2.0f);
			pts[5] = Vec3(desc.box.width /-2.0f, desc.box.length / 2.0f, desc.box.height /-2.0f);
			pts[6] = Vec3(desc.box.width / 2.0f, desc.box.length /-2.0f, desc.box.height /-2.0f);
			pts[7] = Vec3(desc.box.width /-2.0f, desc.box.length /-2.0f, desc.box.height /-2.0f);

			return new ShapeBox(pts, 8);
		}
	case EShape::SHAPE_CONVEX:
		{
			return new ShapeConvex(desc.convex.pts.data(), desc.convex.pts.size());
		}
	}
    return nullptr;
}

ShapeFactoryDesc ShapeFactoryDescHelper::MakeConvex(const std::vector<Vec3>& pts)
{
	ShapeFactoryDesc convex;
	convex.convex.pts = pts;
	return convex;
}

ShapeFactoryDesc ShapeFactoryDescHelper::MakeBox(float width, float length, float height)
{
	ShapeFactoryDesc box;
	box.shapeType = EShape::SHAPE_BOX;
	box.box.height = height;
	box.box.length = length;
	box.box.width = width;

	return box;
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
