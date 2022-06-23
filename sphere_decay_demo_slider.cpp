#include <cstdio>
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "hydro_forces.h"
#include "chrono_irrlicht/ChIrrNodeAsset.h"
#include <chrono>

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;
using namespace chrono;
using namespace chrono::irrlicht;

// =============================================================================

double rest_length = 5;
//double spring_coef = 2e6;
//double damping_coef = 0.1e3;
double ptoC = 0.1e3;
double ptoK = 0.1e6;

// =============================================================================
// Some static data (this is a simple application, we can do this ;) just to allow easy GUI manipulation
IGUIStaticText* text_pto = 0;
// =============================================================================
class MyEventReceiver : public IEventReceiver {
public:

	MyEventReceiver(ChSystemNSC* system, ChIrrAppInterface* myapp, std::shared_ptr<ChBody> body, std::shared_ptr<ChLinkTSDA> spring_1) {
		// store pointer to physical system & other stuff so we can tweak them by user keyboard
		application = myapp;
		msystem = system;
		pto = spring_1;
		mbody = body;
		// ..add a GUI slider to control throttle right via mouse
		scrollbar_ptoC =
			application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 20, 650, 35), 0, 101);
		scrollbar_ptoC->setMax(1e3);
		scrollbar_ptoC->setPos(50);
		text_ptoC =
			application->GetIGUIEnvironment()->addStaticText(L"PTO Damping (Ns/m)", rect<s32>(650, 20, 750, 35), false);

		// ..add a GUI slider to control gas throttle left via mouse
		scrollbar_ptoK =
			application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 45, 650, 60), 0, 102);
		scrollbar_ptoK->setMax(2e6);
		scrollbar_ptoK->setPos(50);
		text_ptoK =
			application->GetIGUIEnvironment()->addStaticText(L"PTO Stiffness (N/m)", rect<s32>(650, 45, 750, 60), false);
	}

	bool OnEvent(const SEvent& event) {
		// check if user moved the sliders with mouse..
		if (event.EventType == EET_GUI_EVENT) {
			s32 id = event.GUIEvent.Caller->getID();

			switch (event.GUIEvent.EventType) {
			case EGET_SCROLL_BAR_CHANGED:
				if (id == 101)  // id of 'motor speed' slider..
				{
					s32 pos1 = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newC = (double)(pos1);
					this->pto->SetDampingCoefficient(newC);
					//auto mfun = std::static_pointer_cast<ChFunction_Const>(pto->GetDampingCoefficient());
					//mfun->Set_yconst(newC);
					return true;
				}
				if (id == 102)  // id of 'motor speed' slider..
				{
					s32 pos2 = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double newK = (double)(pos2);
					this->pto->SetSpringCoefficient(newK);
					//auto mfun =  std::static_pointer_cast<ChFunction_Const>(pto->GetSpringCoefficient());
					//mfun->Set_yconst(newK);
					return true;
				}

				break;
			default:
				break;
			}
		}

		return false;
	}
private:
	ChIrrAppInterface* application;
	ChSystemNSC* msystem;
	std::shared_ptr<ChBody> mbody;
	std::shared_ptr<ChLinkTSDA> pto;
	IGUIStaticText* text_ptoC;
	IGUIStaticText* text_ptoK;
	IGUIScrollBar* scrollbar_ptoC;
	IGUIScrollBar* scrollbar_ptoK;
};




int main(int argc, char* argv[]) {
	auto start = std::chrono::high_resolution_clock::now();
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	ChSystemNSC system;
	system.Set_G_acc(ChVector<>(0, 0, -9.81));

	// Create the Irrlicht application for visualizing
	ChIrrApp application(&system, L"Sphere Decay Test", core::dimension2d<u32>(800, 600), VerticalDir::Z);
	application.AddLogo();
	application.AddSkyBox();
	application.AddTypicalLights();
	application.AddCamera(core::vector3df(0, 30, 0), core::vector3df(0, 0, 0)); // arguments are (location, orientation) as vectors

	// Setup Ground
	auto ground = chrono_types::make_shared<ChBody>();
	system.AddBody(ground);
	ground->SetPos(ChVector<>(0, 0, -5));
	ground->SetIdentifier(-1);
	ground->SetBodyFixed(true);
	ground->SetCollide(false);


	// set up body from a mesh
	std::shared_ptr<ChBody> body = chrono_types::make_shared<ChBodyEasyMesh>(                   //
		GetChronoDataFile("../../HydroChrono/oes_task10_sphere.obj").c_str(),                 // file name
		1000,                                                                                     // density
		false,                                                                                    // do not evaluate mass automatically
		true,                                                                                     // create visualization asset
		false,                                                                                    // do not collide
		nullptr,                                                                                  // no need for contact material
		0                                                                                         // swept sphere radius
		);

	// old sphere stuff (for when you're not using mesh above)
	//std::shared_ptr<ChBody> body = chrono_types::make_shared<ChBodyEasySphere>(5, 1);
	//auto sph = chrono_types::make_shared<ChSphereShape>();
	//body->AddAsset(sph);

	// set up body initial conditions
	system.Add(body);
	body->SetPos(ChVector<>(0, 0, -1));
	body->SetMass(261.8e3);
	body->SetIdentifier(1);
	body->SetBodyFixed(false);
	body->SetCollide(false);

	// attach color asset to body
	auto col_2 = chrono_types::make_shared<ChColorAsset>();
	col_2->SetColor(ChColor(1, 0, 1));
	body->AddAsset(col_2);


	// Create the spring between body_1 and ground. The spring end points are
	// specified in the body relative frames.
	auto spring_1 = chrono_types::make_shared<ChLinkTSDA>();
	spring_1->Initialize(body, ground, true, ChVector<>(0, 0, -1), ChVector<>(0, 0, -5));
	spring_1->SetRestLength(rest_length);
	spring_1->SetSpringCoefficient(ptoK);
	spring_1->SetDampingCoefficient(ptoC);
	system.AddLink(spring_1);
	// attach color asset to spring
	auto col_1 = chrono_types::make_shared<ChColorAsset>();
	col_1->SetColor(ChColor(0, 0, 0));
	spring_1->AddAsset(col_1);
	// Attach a visualization asset.
	spring_1->AddAsset(col_1);
	spring_1->AddAsset(chrono_types::make_shared<ChPointPointSpring>(2, 80, 15));

	// Create a body suspended through a ChLinkTSDA (custom force functor)
	// -------------------------------------------------------------------




	HydroInputs myHydroInputs;
	myHydroInputs.regularWaveAmplitude = 0.022;
	LoadAllHydroForces blah(body, "../../HydroChrono/sphere.h5", myHydroInputs);

	//
	// Prepare some graphical-user-interface (GUI) items to show
	// on the screen
	//

	// ..add a GUI text and GUI slider to control motor of mechanism via mouse

	//IGUIScrollBar* scrollbar =
		//application.GetIGUIEnvironment()->addScrollBar(true, rect<s32>(300, 105, 450, 120), 0, 101);
	//scrollbar->setMax(100);

	// Use this function for adding a ChIrrNodeAsset to all items
	// Otherwise use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();

	// Use this function for 'converting' assets into Irrlicht meshes
	application.AssetUpdateAll();

	//
  // USER INTERFACE
  //

  // Create some graphical-user-interface (GUI) items to show on the screen.
  // This requires an event receiver object.

	MyEventReceiver receiver( &system, &application, body, spring_1);

	// note how to add the custom event receiver to the default interface:
	application.SetUserEventReceiver(&receiver);


	// Info about which solver to use - may want to change this later
	auto gmres_solver = chrono_types::make_shared<ChSolverGMRES>();  // change to mkl or minres?
	gmres_solver->SetMaxIterations(300);
	system.SetSolver(gmres_solver);
	double timestep = 0.015; // also sets the timesteps in chrono system
	application.SetTimestep(timestep);

	// set up output file for body position each step
	std::string of = "output.txt";                    /// < put name of your output file here
	std::ofstream zpos(of, std::ofstream::out);
	if (!zpos.is_open()) {
		std::cout << "Error opening file \"" + of + "\". Please make sure this file path exists then try again\n";
		return -1;
	}
	zpos.precision(10);
	zpos.width(12);
	zpos << "#Time\tBody Pos\tBody vel (heave)\tforce (heave)\tSpring Length (m)\tSpring Velocity (m/s)\tSpring Force (N)\n";

	// Simulation loop
	int frame = 0;
	while (application.GetDevice()->run() && system.GetChTime() <= 1000) {
		application.BeginScene();
		application.DrawAll();
		/*if (buttonPressed)*/if (true) {
			zpos << system.GetChTime() << "\t" << body->GetPos().z() << "\t" << body->GetPos_dt().z() << "\t" << body->GetAppliedForce().z() << "\t" << spring_1->GetLength() << "\t" << spring_1->GetVelocity() << "\t" << spring_1->GetForce() << "\n";
			application.DoStep();
			frame++;
		}
		application.EndScene();
	}
	zpos.close();
	auto end = std::chrono::high_resolution_clock::now();
	unsigned duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	std::cout << "Duration: " << duration / 1000.0 << " seconds" << std::endl;
	return 0;
}