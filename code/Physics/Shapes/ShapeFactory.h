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

	struct
	{
		float width, length, height;
	} box;

	struct
	{
		std::vector<Vec3> pts;
	} convex;
};

struct ShapeFactoryDescHelper
{
	static ShapeFactoryDesc MakeConvex(const std::vector<Vec3>& pts);
	static ShapeFactoryDesc MakeBox(float width, float length, float height);
	static ShapeFactoryDesc MakeSphere(float radius);
	static ShapeFactoryDesc MakePlane(float width, float height);
};


class ShapeFactory
{
public:

	ShapeFactory();

	Shape* CreateShape(const ShapeFactoryDesc& desc);

};
