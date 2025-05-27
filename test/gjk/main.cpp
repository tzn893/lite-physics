#include "Math/Vector.h"
#include "Physics/GJK.h"
#include "gtest/gtest.h"


#define ASSERT_FLOAT_APPROX_EQ(lhs, rhs) EXPECT_NEAR(lhs, rhs, 5e-4)

#define AssertEqualVec4(lhs, rhs)\
{\
	ASSERT_FLOAT_APPROX_EQ(lhs.x, rhs.x);\
	ASSERT_FLOAT_APPROX_EQ(lhs.y, rhs.y);\
	ASSERT_FLOAT_APPROX_EQ(lhs.z, rhs.z);\
	ASSERT_FLOAT_APPROX_EQ(lhs.w, rhs.w);\
}

#define AssertEqualVec3(lhs, rhs)\
{\
	ASSERT_FLOAT_APPROX_EQ(lhs.x, rhs.x);\
	ASSERT_FLOAT_APPROX_EQ(lhs.y, rhs.y);\
	ASSERT_FLOAT_APPROX_EQ(lhs.z, rhs.z);\
}

#define AssertEqualVec2(lhs, rhs)\
{\
	ASSERT_FLOAT_APPROX_EQ(lhs.x, rhs.x);\
	ASSERT_FLOAT_APPROX_EQ(lhs.y, rhs.y);\
}


TEST(TestSignedVolumeProjection, TestSignedVolumeProjection)
{
	const Vec3 orgPts[4] = {
	Vec3(0, 0, 0) ,
	Vec3(1, 0, 0) ,
	Vec3(0, 1, 0) ,
	Vec3(0, 0, 1) ,
	};
	Vec3 pts[4];
	Vec4 lambdas;
	Vec3 v;

	pts[0] = orgPts[0] + Vec3(-0.2,-0.4, 0.0);
	pts[1] = orgPts[1] + Vec3(-0.2,-0.4, 0.0);
	pts[2] = orgPts[2] + Vec3(-0.2,-0.4, 0.0);
	pts[3] = orgPts[3];

	lambdas = SignedVolume(pts[0], pts[1], pts[2], pts[3]);
	v.Zero();
	for (int i = 0; i < 4; i++) {
		v += pts[i] * lambdas[i];
	}
	AssertEqualVec4(lambdas, Vec4(0.4f, 0.2f, 0.4f, 0.0f));
	AssertEqualVec3(v, Vec3(0, 0, 0));
	

	for (int i = 0; i < 4; i++) {
		pts[i] = orgPts[i] + Vec3(1, 1, 1);
	}
	lambdas = SignedVolume(pts[0], pts[1], pts[2], pts[3]);
	v.Zero();
	for (int i = 0; i < 4; i++) {
		v += pts[i] * lambdas[i];
	}
	AssertEqualVec4(lambdas, Vec4(1.0f, 0.0f, 0.0f, 0.0f));
	AssertEqualVec3(v, Vec3(1, 1, 1));

	for (int i = 0; i < 4; i++) {
		pts[i] = orgPts[i] + Vec3(-1,-1,-1) * 0.25f;
	}
	lambdas = SignedVolume(pts[0], pts[1], pts[2], pts[3]);
	v.Zero();
	for (int i = 0; i < 4; i++) {
		v += pts[i] * lambdas[i];
	}
	AssertEqualVec4(lambdas, Vec4(0.250, 0.250, 0.250, 0.250));
	AssertEqualVec3(v, Vec3(0.000, 0.000, 0.000));

	for (int i = 0; i < 4; i++) {
		pts[i] = orgPts[i] + Vec3(-1,-1,-1);
	}
	lambdas = SignedVolume(pts[0], pts[1], pts[2], pts[3]);
	v.Zero();
	for (int i = 0; i < 4; i++) {
		v += pts[i] * lambdas[i];
	}
	AssertEqualVec4(lambdas, Vec4(0.000, 0.33333, 0.333333, 0.333333));
	AssertEqualVec3(v, Vec3(-0.66667, -0.66667, -0.66667));

	for (int i = 0; i < 4; i++) {
		pts[i] = orgPts[i] + Vec3(1, 1, -0.5f);
	}
	lambdas = SignedVolume(pts[0], pts[1], pts[2], pts[3]);
	v.Zero();
	for (int i = 0; i < 4; i++) {
		v += pts[i] * lambdas[i];
	}
	AssertEqualVec4(lambdas, Vec4(0.500, 0.000, 0.000, 0.500));
	AssertEqualVec3(v, Vec3(1.000, 1.000, 0.000));

	pts[0] = Vec3(51.1996613f, 26.1989613f, 1.91339576f);
	pts[1] = Vec3(-51.0567360f, -26.0565681f, -0.436143428f);
	pts[2] = Vec3(50.8978920f, -24.1035538f, -1.04042661f);
	pts[3] = Vec3(-49.1021080f, 25.8964462f, -1.04042661f);
	lambdas = SignedVolume(pts[0], pts[1], pts[2], pts[3]);
	v.Zero();
	for (int i = 0; i < 4; i++) {
		v += pts[i] * lambdas[i];
	}
	AssertEqualVec4(lambdas, Vec4(0.290, 0.302, 0.206, 0.202));
	AssertEqualVec3(v, Vec3(0, 0, 0));

	pts[0] = Vec3(6, 0, 0);
	pts[1] = Vec3(-4, 0, 0);
	Vec2 lambda2D = SignedVolume(pts[0], pts[1]);
	v.Zero();
	for (int i = 0;i < 2;i++)
	{
		v += pts[i] * lambda2D[i];
	}
	AssertEqualVec2(lambda2D, Vec2(0.4, 0.6));
	AssertEqualVec3(v, Vec3(0, 0, 0));

}

int main() 
{
	testing::InitGoogleTest();
	return RUN_ALL_TESTS();
}