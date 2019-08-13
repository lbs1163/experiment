//==============================================================================
/*
Texture Surface Rendering Program using only Force-feedback / LSF model

This is a program for user study, to compare FF-only feedback and LSF vibrotactile feedback.
The texture information are modeled in heights maps. Also use the Dahl model and the Hunt-Crossley
model to render the friction and the stiffness characteristic of surfaces.
When using LSF, this program doesn't use geometry data. instead, render vibration generated by LSF model.

For LSF modeling and rendering, we got help from Arsen Abdulali, at KHU.

Sunghwan Shin, 2018

*/
//==============================================================================
//------------------------------------------------------------------------------
#include "chai3d.h"
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include "shared.h"
#include "sharedInit.h"
#include <direct.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "IconsFontAwesome5.h"

#define NUM_TRAINING_SESSION 2
#define NUM_EXPERIMENT_SESSION 3
#define NUM_SESSION 8
#define NUM_TRIAL 40

//-------------------
//Experiment setting
//-------------------
struct ExpSessionResponse responseArray[NUM_SESSION][NUM_TRIAL];
int objectNumberArray[NUM_SESSION][NUM_TRIAL];
int currentSession = 0;
int currentTrial = 0;






//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// fullscreen mode
bool fullscreen = false;

//**********
// My constants
//Vibration model
//VibMaker* myVM;

//**********
//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

double toolRadius = 0.0;

cFontPtr font;
ImFont* GUIfont;
ImFont* GUIfont2;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;
cLabel* labelMovingStatus;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

const char* glsl_version = "#version 150";

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

int numTotalSubject;
int numCurrentSubject;
char** subjectItems;

// root resource path
double workspaceScaleFactor;
double maxLinearForce;
double maxLinearDamping;
double maxStiffness;

//texture object
cMesh* objectArray[NUM_SIGMA * NUM_ZMAX * NUM_ZRATIO];

bool isSampleReady = false;
bool isTraining = true;
bool goToNextTexture = false;
bool showControlPanel = false;
int trainingTexture = 0;

double forceMagnitude = 0;

//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

template <typename myT> int sgn(myT val) {
    return (myT(0) < val) - (val < myT(0));
}

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------
// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// callback to render graphic scene
void updateGraphics(void);
void updateGUI(void);

// function that closes the application
void close(void);

// main haptics simulation loop
void updateHaptics(void);

int loadTexture();

int exp_next();
int exp_prev();
void exp_start();
void exp_samp_change();
void exp_save_response();


double sigmaArray[NUM_SIGMA] = { 1.0E+1, 2.5E+1, 6.0E+1, 1.5E+2, 3.9E+2 };
double zMaxArray[NUM_ZMAX] = { 3.0E-5, 1.0E-4, 3.0E-4, 1.0E-3, 3.0E-3, 1.0E-2 };
double zRatioArray[NUM_ZRATIO] = { 1.1, 1.2, 1.3, 1.4 };


int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "Friction absolute magnitude estimation experiment" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);

    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    //glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    glfwMaximizeWindow(window);

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    //glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif

    // GUI init
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
                                                           //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls
    io.Fonts->AddFontDefault();
    GUIfont = io.Fonts->AddFontFromFileTTF("../../resources/fonts/NanumGothicBold.ttf", 20.0f, NULL, io.Fonts->GetGlyphRangesKorean());
    ImFontConfig config;
    config.MergeMode = true;
    config.GlyphMinAdvanceX = 13.0f; // Use if you want to make the icon monospaced
    static const ImWchar icon_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
    io.Fonts->AddFontFromFileTTF("../../resources/fonts/fa-solid-900.ttf", 20.0f, &config, icon_ranges);
    GUIfont2 = io.Fonts->AddFontFromFileTTF("../../resources/fonts/NanumGothicBold.ttf", 40.0f, NULL, io.Fonts->GetGlyphRangesKorean());


    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    ImGuiStyle& myStyle = ImGui::GetStyle();
    myStyle.WindowTitleAlign = ImVec2(0.5f, 0.5f);

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);



    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(0.15, 0.0, 0.2),    // camera position (eye)
        cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
        cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

                                     // set the near and far clipping planes of the camera
                                     // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(C_STEREO_DISABLED);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(false);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos(0.0, 0.0, 0.7);

    // define the direction of the light beam
    light->setDir(0.0, 0.0, -1.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityVeryHigh();

    // set light cone half angle
    light->setCutOffAngleDeg(40);

    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // if the device has a gripper, then enable it to behave like a user switch
    hapticDevice->setEnableGripperUserSwitch(true);

    // create a 3D tool and add it to the camera
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, true);

    // set color of tool
    tool->m_hapticPoint->m_sphereProxy->m_material->setBlack();

    // set radius of tool
    toolRadius = 0.0015;

    // define a radius for the tool
    tool->setRadius(toolRadius);

    //tool->setShowContactPointsFriction(true, true, true, cColorf(0.0, 0.0, 0.0));

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    tool->setWorkspaceScaleFactor(1.0);
    workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // get properties of haptic device
    maxLinearForce = hapticDeviceInfo.m_maxLinearForce;
    maxLinearDamping = hapticDeviceInfo.m_maxLinearDamping;
    maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // initialize tool by connecting to haptic device
    tool->start();


    // If delta devices, set effectoor mass
    if (hapticDeviceInfo.m_model >= C_HAPTIC_DEVICE_DELTA_3 && hapticDeviceInfo.m_model <= C_HAPTIC_DEVICE_SIGMA_7)
    {
        cDeltaDevicePtr deltaDevice = std::dynamic_pointer_cast<cDeltaDevice>(hapticDevice);
        if (deltaDevice->setEffectorMass(0.335))
        {
            cout << "EFM changed." << endl;
        }
    }

    /////////////////////////////////////////////////////////////////////////
    // Texture loading
    /////////////////////////////////////////////////////////////////////////

    //Texture info
    loadTexture();

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    labelMovingStatus = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);
    camera->m_frontLayer->addChild(labelMovingStatus);

    // set font color
    labelHapticRate->m_fontColor.setBlack();
    labelMovingStatus->m_fontColor.setBlack();

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
        cColorf(1.0f, 1.0f, 1.0f),
        cColorf(0.8f, 0.8f, 0.8f),
        cColorf(0.8f, 0.8f, 0.8f));

    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------
    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    //output = fopen("output.txt", "w");
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);

    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render CHAI3D graphics and GUI
        updateGraphics();
        updateGUI();

        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;

    labelHapticRate->setLocalPos(20, height - 40, 0);
    labelMovingStatus->setLocalPos(20, height - 80, 0);
}

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_X))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    else if (a_key == GLFW_KEY_C)
    {
        showControlPanel = !showControlPanel;
    }

    else if (a_key == GLFW_KEY_SPACE)
    {
        isSampleReady = !isSampleReady;
    }

    else if (a_key == GLFW_KEY_S)
    {
        exp_save_response();
    }

	else if (a_key == GLFW_KEY_ENTER)
	{
		goToNextTexture = true;
	}
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
    //	delete myVM;


    for (int i = 0; i < numTotalSubject; i++)
        delete subjectItems[i];
}

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic rate data
    labelHapticRate->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    if (vibError)
        monitor = 1;

    string movingStatus;
    movingStatus = std::to_string(monitor) + "~" + std::to_string(fN) + "-" + std::to_string(vT) + "-" + std::to_string(cA);
    labelMovingStatus->setText(movingStatus);

    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (width - labelHapticRate->getWidth())), 15);
    labelMovingStatus->setLocalPos((int)(0.5 * (width - labelHapticRate->getWidth())), 30);

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, false);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------


void updateGUI(void)
{
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ImGui::PushFont(GUIfont);


    if (!experimentRunning)	//Start GUI
    {
        ImGui::Begin("Friction Experiment", 0, ImGuiWindowFlags_AlwaysAutoResize);                          // Create a window and append into it.
        ImGui::Text("                                     ");
        ImGui::Text("Subject");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        ImGui::Combo("##Subject", &numCurrentSubject, subjectItems, numTotalSubject);
        ImVec2 size = ImGui::GetItemRectSize();
        ImVec2 size2 = ImGui::GetContentRegionAvail();
        ImGui::Dummy(ImVec2((size2.x - 3.0 * ImGui::GetStyle().ItemSpacing.x - 160) / 2.0, 0.0f));
        ImGui::SameLine();
        if (ImGui::Button("Start", ImVec2(80, size.y)))
            ImGui::OpenPopup("Training Session");
               
        if (ImGui::BeginPopupModal("Training Session", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Four training trials will be given");
            ImGui::Text("after press the OK button below.");
            ImVec2 size3 = ImGui::GetContentRegionAvail();
            ImGui::Dummy(ImVec2((size3.x - 80 - 2 * ImGui::GetStyle().ItemSpacing.x) / 2.0, size3.y));
            ImGui::SameLine();
            if (ImGui::Button("OK", ImVec2(80, size.y)))
            {
                ImGui::CloseCurrentPopup();
                exp_start();
            }
            ImGui::EndPopup();
        }

        ImGui::End();
    }

    else	// Experiment running GUI
    {
        ImGui::Begin("Friction Exp.", 0, ImGuiWindowFlags_AlwaysAutoResize);                          // Create a window and append into it.
        ImGui::BeginGroup();

        ImGui::AlignTextToFramePadding(); ImGui::Text("Friction");
        ImGui::EndGroup();
        ImVec2 size = ImGui::GetItemRectSize();
        ImGui::SameLine();

        ImGui::BeginGroup();
        ImGui::PushItemWidth(size.x);
		if (ImGui::IsRootWindowOrAnyChildFocused() && !ImGui::IsAnyItemActive() && !ImGui::IsMouseClicked(0))
			ImGui::SetKeyboardFocusHere(0);
		ImGui::InputText("##F", responseArray[currentSession][currentTrial].resp.friction, 64);
        ImGui::PopItemWidth();
        ImGui::EndGroup();
        ImGui::Dummy(ImVec2(0.0f, 20.0f));
        ImVec2 size2 = ImGui::GetContentRegionAvail();
        if (ImGui::Button("Prev\n  " ICON_FA_ARROW_ALT_CIRCLE_LEFT, ImVec2((size2.x - ImGui::GetStyle().ItemSpacing.x)*0.5f, size.y * 2)))
        {
            int result;
            result = exp_prev();
            switch (result)
            {
            case 0:
                ImGui::OpenPopup("Error2");
                break;
            case -1:
                ImGui::OpenPopup("Error1");
                break;
            }
        }
        ImGui::SameLine();
        if (goToNextTexture || ImGui::Button("Next\n  " ICON_FA_ARROW_ALT_CIRCLE_RIGHT, ImVec2((size2.x - ImGui::GetStyle().ItemSpacing.x)*0.5f, size.y * 2)))
        {
            int result;
			goToNextTexture = false;
            result = exp_next();
            switch (result)
            {
            case 3:
                isSampleReady = true;
                ImGui::OpenPopup("End of the training");
                break;
            case 0:
                ImGui::OpenPopup("End of the experiment");
                break;
            case -1:
                ImGui::OpenPopup("Error1");
                break;
            }
        }
        if (!isSampleReady)
            ImGui::OpenPopup("Texture Changing...");
        if (ImGui::BeginPopupModal("Error1", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("You can't change textures while you touching them.");
            ImVec2 size3 = ImGui::GetContentRegionAvail();
            ImGui::Dummy(ImVec2((size3.x - 80 - 2 * ImGui::GetStyle().ItemSpacing.x) / 2.0, 0));
            ImGui::SameLine();
            if (ImGui::Button("OK", ImVec2(80, 30)))
            {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
        if (ImGui::BeginPopupModal("Error2", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("There is no previous texture.");
            ImVec2 size3 = ImGui::GetContentRegionAvail();
            ImGui::Dummy(ImVec2((size3.x - 80 - 2 * ImGui::GetStyle().ItemSpacing.x) / 2.0, 0));
            ImGui::SameLine();
            if (ImGui::Button("OK", ImVec2(80, 30)))
            {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
        if (ImGui::BeginPopupModal("End of the experiment", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("This is the end of the experiment. Thank you.");
            ImVec2 size3 = ImGui::GetContentRegionAvail();
            ImGui::Dummy(ImVec2((size3.x - 80 - 2 * ImGui::GetStyle().ItemSpacing.x) / 2.0, 0));
            ImGui::SameLine();
            if (ImGui::Button("OK", ImVec2(80, 30)))
            {
                ImGui::CloseCurrentPopup();
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
            ImGui::EndPopup();
        }
        if (ImGui::BeginPopupModal("End of the training", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("After you press the button, main session will be started.");
            ImVec2 size3 = ImGui::GetContentRegionAvail();
            ImGui::Text("Wait for the texture to be set.");
            ImGui::Dummy(ImVec2((size3.x - 80 - 2 * ImGui::GetStyle().ItemSpacing.x) / 2.0, 0));
            ImGui::SameLine();
            if (ImGui::Button("OK", ImVec2(80, 30)))
            {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
        if (ImGui::BeginPopupModal("Texture Changing...", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Wait for the texture to be set.");
            if (isSampleReady)
            {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
        ImGui::End();

        if (showControlPanel)
        {

            float sigma = objectArray[trainingTexture]->m_material->getSigma();
            float zmax = objectArray[trainingTexture]->m_material->getZmax();
            float zstick = objectArray[trainingTexture]->m_material->getZstick();
			float zratio = zstick / zmax;
            string parameterInformationString;
            ostringstream strs;
			strs << "Sigma:" << sigma << "    zMax:" << zmax << "   zStick:" << zstick;
			parameterInformationString = strs.str();

            string forceMagnitudeText;
            ostringstream strs2;
            strs2 << forceMagnitude;
            forceMagnitudeText = "Force Magnitude: " + strs2.str();

            
            ImGui::Begin("Friction Control Panel", 0, ImGuiWindowFlags_AlwaysAutoResize);                          // Create a window and append into it.
            ImGui::SameLine();
            ImGui::SetNextItemWidth(2000);
            ImGui::BeginGroup();

			if (ImGui::Button("10", ImVec2(80, 30)))
				sigma = 10;
			ImGui::SameLine();
			if (ImGui::Button("25", ImVec2(80, 30)))
				sigma = 25;
			ImGui::SameLine();
			if (ImGui::Button("60", ImVec2(80, 30)))
				sigma = 60;
			ImGui::SameLine();
			if (ImGui::Button("150", ImVec2(80, 30)))
				sigma = 150;
			ImGui::SameLine();
			if (ImGui::Button("390", ImVec2(80, 30)))
				sigma = 390;
			ImGui::SameLine();
			ImGui::Text("Sigma");

			
			if (ImGui::Button("3e-5", ImVec2(80, 30)))
				zmax = 3e-5; zstick = zmax*zratio;
			ImGui::SameLine();
			if (ImGui::Button("1e-4", ImVec2(80, 30)))
				zmax = 1e-4; zstick = zmax*zratio;
			ImGui::SameLine();
			if (ImGui::Button("3e-4", ImVec2(80, 30)))
				zmax = 3e-4; zstick = zmax*zratio;
			ImGui::SameLine();
			if (ImGui::Button("1e-3", ImVec2(80, 30)))
				zmax = 1e-3; zstick = zmax*zratio;
			ImGui::SameLine();
			if (ImGui::Button("3e-3", ImVec2(80, 30)))
				zmax = 3e-3; zstick = zmax*zratio;
			ImGui::SameLine();
			if (ImGui::Button("1e-2", ImVec2(80, 30)))
				zmax = 1e-2; zstick = zmax*zratio;
			ImGui::SameLine();
			ImGui::Text("zMax");


			if (ImGui::Button("1.1", ImVec2(80, 30)))
				zstick = 1.1*zmax;
			ImGui::SameLine();
			if (ImGui::Button("1.2", ImVec2(80, 30)))
				zstick = 1.2*zmax;
			ImGui::SameLine();
			if (ImGui::Button("1.3", ImVec2(80, 30)))
				zstick = 1.3*zmax;
			ImGui::SameLine();
			if (ImGui::Button("1.4", ImVec2(80, 30)))
				zstick = 1.4*zmax;
			ImGui::SameLine();
			ImGui::Text("zRatio");

			ImGui::Text(parameterInformationString.c_str());

            ImGui::EndGroup();
            ImGui::SetNextItemWidth((float)size.x * 2 + ImGui::GetStyle().ItemSpacing.x);
            ImGui::End();

            ImGui::Begin("Friction Control Panel 2", 0, ImGuiWindowFlags_AlwaysAutoResize);                          // Create a window and append into it.
            ImGui::SameLine();
            ImGui::SetNextItemWidth(2000);
            ImGui::BeginGroup();
            ImGui::SliderFloat("Sigma", &sigma, 10.0f, 5000.0f, "%.4e");
            size = ImGui::GetItemRectSize();
            ImGui::SetNextItemWidth(2000);
            ImGui::SliderFloat("zMax", &zmax, 0.00001f, 0.1f, "%.4e");
            ImGui::SetNextItemWidth(2000);
            ImGui::SliderFloat("zStick", &zstick, 0.00001f, 0.1f, "%.4e");
            ImGui::Text(forceMagnitudeText.c_str());
            ImGui::EndGroup();
            ImGui::SetNextItemWidth((float)size.x * 2 + ImGui::GetStyle().ItemSpacing.x);
            ImGui::End();
            
            objectArray[trainingTexture]->m_material->setSigma(sigma);
            objectArray[trainingTexture]->m_material->setZmax(zmax);
            objectArray[trainingTexture]->m_material->setZstick(zstick);
        }

        ImGui::SetNextWindowPos(ImVec2(750, 200));
        ImGui::SetNextWindowSize(ImVec2(420, 100));
        ImGui::Begin("Exp. Progress", 0, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::PushFont(GUIfont2);
        string progress;

		if (isTraining)
			progress = "Training " + std::to_string(currentSession + 1) + "-" + std::to_string(currentTrial + 1) + "/" + std::to_string(NUM_SIGMA * NUM_ZMAX * NUM_ZRATIO / 3);
		else
			progress = "Main Exp. " + std::to_string(currentSession - 1) + "-" + std::to_string(currentTrial + 1) + "/" + std::to_string(NUM_SIGMA * NUM_ZMAX * NUM_ZRATIO / 3);
        ImVec2 textSize = ImGui::CalcTextSize(progress.c_str());
        ImGui::SetCursorPosX((420 - textSize.x) / 2);
        ImGui::Text(progress.c_str());
        ImGui::PopFont();
        ImGui::End();
    }
    ImGui::PopFont();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{   // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    ofstream log("log.txt");

    // main haptic simulation loop
    while (simulationRunning)
    {
        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        bool outputRender = false;

        if (experimentRunning && isSampleReady)	//Render force and vibration only when the experiment is in running
        {
            if (tool->isInContact(objectArray[trainingTexture]))
                outputRender = true;
        }

        if (outputRender)
        {
            cVector3d normal, force, vel, velN, velT;

            normal = tool->getHapticPoint(0)->getCollisionEvent(0)->m_globalNormal;
            force = tool->getDeviceGlobalForce();
            vel = tool->getDeviceGlobalLinVel();
            fN = force.dot(normal);
            normal.mulr(vel.dot(normal), velN);
            vel.subr(velN, velT);
            vT = velT.length();

            force = tool->getHapticPoint(0)->m_algorithmFingerProxy->getTangentialForce();

            //log << force << endl;
            /*
            if (vT > velTH && dispVib)
            {
            myVM->updateHapticInformation(fN, vT);
            }
            else if (dispVib)
            {
            myVM->stop();
            }
            */
        }
        else
        {
            tool->setDeviceGlobalForce(0.0, 0.0, 0.0);
            //			myVM->stop();
            fN = 0.0;
            vT = 0.0;
        }
        cVector3d normal = cVector3d(0, 0, 1);
        cVector3d force = tool->getDeviceGlobalForce();
        cVector3d lateralForce;
        normal.mul(force.dot(normal));
        force.subr(normal, lateralForce);
        forceMagnitude = lateralForce.length();

        if (!experimentRunning)
            force.add(0, 0, -1.65);
        else
            force.add(0, 0, -2.50);

        tool->setDeviceGlobalForce(force);

        tool->applyToDevice();
        // update frequency counter
        freqCounterHaptics.signal(1);
    }
    log.close();

    // exit haptics thread
    simulationFinished = true;
}



int loadTexture()
{
    int result = 0;

	for (int sigmaIndex = 0; sigmaIndex < NUM_SIGMA; sigmaIndex++)
		for (int zMaxIndex = 0; zMaxIndex < NUM_ZMAX; zMaxIndex++)
			for (int zRatioIndex = 0; zRatioIndex < NUM_ZRATIO; zRatioIndex++)
			{
				double sigma = sigmaArray[sigmaIndex];
				double zMax = zMaxArray[zMaxIndex];
				double zStick = zMax * zRatioArray[zRatioIndex];
				int objectIndex = sigmaIndex * NUM_ZMAX * NUM_ZRATIO + zMaxIndex * NUM_ZRATIO + zRatioIndex;

				objectArray[objectIndex] = new cMesh();
				
				cCreatePlane(objectArray[objectIndex], planeSize, planeSize);
				objectArray[objectIndex]->createAABBCollisionDetector(toolRadius);
				world->addChild(objectArray[objectIndex]);
				objectArray[objectIndex]->setLocalPos(planeX, 0, planeZ);
				///////////////////
				objectArray[objectIndex]->m_texture = cTexture2d::create();
				//////////////////
				objectArray[objectIndex]->m_material->setWhite();

				objectArray[objectIndex]->m_material->setStiffness(maxStiffness);
				objectArray[objectIndex]->m_material->setSigma(sigma);
				objectArray[objectIndex]->m_material->setZmax(zMax);
				objectArray[objectIndex]->m_material->setZstick(zStick);
				objectArray[objectIndex]->m_material->setHapticTriangleSides(true, false);

				objectArray[objectIndex]->setEnabled(false, true);
			}

	int temp[NUM_SIGMA * NUM_ZMAX * NUM_ZRATIO];
	for (int index = 0; index < NUM_SIGMA * NUM_ZMAX * NUM_ZRATIO; index++)
		temp[index] = index;
	random_shuffle(std::begin(temp), std::end(temp));
	memcpy(objectNumberArray[0], temp, 80 * sizeof(int));
	random_shuffle(std::begin(temp), std::end(temp));
	memcpy(objectNumberArray[2], temp, 120 * sizeof(int));
	random_shuffle(std::begin(temp), std::end(temp));
	memcpy(objectNumberArray[5], temp, 120 * sizeof(int));

    return result;
}

int exp_prev()
{
	if (!(tool->isInContact(objectArray[trainingTexture])))
	{
		if (currentTrial > 0)
		{
			currentTrial--;
			exp_samp_change();
			// Successful change
			return 1;
		}
		else
		{
			// First sample, can not go foward
			return 0;
		}
	}
	else
	{
		// Now touching
		return -1;
	}
}

int exp_next()
{
	if (!(tool->isInContact(objectArray[trainingTexture])))
	{
		currentTrial++;
		if (currentTrial >= NUM_TRIAL)
		{
			currentSession++;
			currentTrial = 0;
			if (currentSession >= NUM_SESSION)
			{
				// End of the experiment
				currentSession--;
				currentTrial = NUM_TRIAL - 1;
				exp_save_response();
				return 0;
			}

			exp_samp_change();
			if (currentSession == NUM_TRAINING_SESSION)
			{
				// End of the training
				isTraining = false;
				return 3;
			}
			// Successful session change
			return 2;
		}
		else
		{
			// Successful trial change
			exp_samp_change();
			return 1;
		}
	}
	else
		return -1;
}

void exp_start()
{
    if (!experimentRunning)
    {
        experimentRunning = true;
        cout << "Experiment started." << endl;
        exp_samp_change();
        //	myVM->startVibration();
    }
}

void exp_samp_change()
{
    isSampleReady = true;

    //Deactivate the previous models
	objectArray[trainingTexture]->setEnabled(false, true);
	trainingTexture = objectNumberArray[currentSession][currentTrial];
	objectArray[trainingTexture]->setEnabled(true, true);
	cout << scientific;
	cout << "sigma: " << objectArray[trainingTexture]->m_material->getSigma() << " z_max: " << objectArray[trainingTexture]->m_material->getZmax() 
		<< " z_stick: " << objectArray[trainingTexture]->m_material->getZstick() << endl << endl;
}


void exp_save_response()
{
    string filename;
    filename = "exp/R" + std::to_string(numCurrentSubject + 1) + ".txt";
    ofstream response(filename);
	vector<pair<int, string>> responseVector;

	for (int experimentIndex = 0; experimentIndex < NUM_REPETITION; experimentIndex++)
	{
		for (int sessionCount = 0; sessionCount < NUM_EXPERIMENT_SESSION; sessionCount++)
			for (int trialIndex = 0; trialIndex < NUM_TRIAL; trialIndex++)
			{
				int sessionIndex = NUM_TRAINING_SESSION + experimentIndex * NUM_EXPERIMENT_SESSION + sessionCount;
				int objectNumber = objectNumberArray[sessionIndex][trialIndex];
				string responseValue(responseArray[sessionIndex][trialIndex].resp.friction);
				responseVector.push_back(pair<int, string>(objectNumber, responseValue));
			}
		sort(responseVector.begin(), responseVector.end());
		for (auto iter = responseVector.begin(); iter != responseVector.end(); iter++)
			response << "[" << iter->first << ":" << iter->second << "]" << endl;
	}

    response.close();
}
