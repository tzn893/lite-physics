//
//  application.h
//
#pragma once
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <deque>

#include "Math/Vector.h"
#include "Math/Quat.h"
#include "Physics/Shapes.h"
#include "Physics/Body.h"

#include "Renderer/DeviceContext.h"
#include "Renderer/model.h"
#include "Renderer/shader.h"
#include "Renderer/FrameBuffer.h"

#include "Physics/Scene.h"


#define DEFINE_APPLICATION_ENTRANCE(App) int main( int argc, char * argv[] ) {\
	g_application = new App;\
	g_application->Initialize();\
	\
	g_application->MainLoop();\
	\
	delete g_application;\
	return 0;\
}


/*
====================================================
Application
====================================================
*/

enum EInputKey
{
	IK_W = 0,
	IK_A,
	IK_S,
	IK_D,
	IK_Q,
	IK_E,
	IK_F,
	IK_G,
	IK_H,
	IK_J,
	IK_K,
	IK_L,
	IK_I,
	IK_O,
	IK_COUNT
};


class InputBuffer
{
	friend class Application;

	void KeyPress(EInputKey key);
	void KeyRelease(EInputKey key);
	void Update();

	enum EInputKeyState
	{
		EIS_PRESSED,
		EIS_RELEASED,
		EIS_HOLD,
		EIS_NONE,
	};

	EInputKeyState m_keyState[IK_COUNT];

public:
	InputBuffer();

	bool GetKeyDown(EInputKey inputKey);

	bool GetKeyHold(EInputKey inputKey);

	bool GetKeyUp(EInputKey inputKey);
};

class Application {
public:
	Application() : m_isPaused( true ), m_stepFrame( false ) {}
	virtual ~Application();

	void Initialize();
	void MainLoop();

	virtual void BuildScene(class SceneBuilder* builder);
	virtual void UpdateScene(float dt_sec);

protected:
	std::vector< const char * > GetGLFWRequiredExtensions() const;

	void InitializeGLFW();
	bool InitializeVulkan();
	void Cleanup();
	void UpdateUniforms();
	void DrawFrame();
	void ResizeWindow( int windowWidth, int windowHeight );
	void MouseMoved( float x, float y );
	void MouseScrolled( float z );
	void Keyboard( int key, int scancode, int action, int modifiers );


	virtual void Start() {}

	static void OnWindowResized( GLFWwindow * window, int width, int height );
	static void OnMouseMoved( GLFWwindow * window, double x, double y );
	static void OnMouseWheelScrolled( GLFWwindow * window, double x, double y );
	static void OnKeyboard( GLFWwindow * window, int key, int scancode, int action, int modifiers );

protected:
	Scene * m_scene;

	GLFWwindow * m_glfwWindow;

	DeviceContext m_deviceContext;

	InputBuffer m_inputBuffer;

	//
	//	Uniform Buffer
	//
	Buffer m_uniformBuffer;

	//
	//	Model
	//
	Model m_modelFullScreen;
	std::vector< Model * > m_models;	// models for the bodies

	//
	//	Pipeline for copying the offscreen framebuffer to the swapchain
	//
	Shader		m_copyShader;
	Descriptors	m_copyDescriptors;
	Pipeline	m_copyPipeline;

	// User input
	Vec2 m_mousePosition;
	Vec3 m_cameraFocusPoint;
	float m_cameraPositionTheta;
	float m_cameraPositionPhi;
	float m_cameraRadius;
	bool m_isPaused;
	bool m_stepFrame;

	std::vector< RenderModel > m_renderModels;

	static const int WINDOW_WIDTH = 1200;
	static const int WINDOW_HEIGHT = 720;

	static const bool m_enableLayers = true;


	int maxSceneStateCounter = 1;
	int sceneStateCounter = 0;

	int maxSceneStateCnt = 1000;
	std::deque<SceneState> m_sceneStates;
};

extern Application * g_application;