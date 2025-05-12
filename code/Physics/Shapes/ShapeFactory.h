#pragma once


#include "../Shapes.h"



struct ShapeFactoryDesc
{
	Shape::shapeType_t shapeType;

	struct
	{
		float radius;
	} sphere;

	struct
	{
		float width, height;
	} plane;
};

struct ShapeFactoryDescHelper
{
	static ShapeFactoryDesc MakeSphere(float radius);
	static ShapeFactoryDesc MakePlane(float width, float height);
};


class ShapeFactory
{
public:

	ShapeFactory();

	Shape* CreateShape(const ShapeFactoryDesc& desc);

};
