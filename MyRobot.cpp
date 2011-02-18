#include "WPILib.h"
#include "math.h"

/*
 * Drive Motor 1 (FL) - Jaguar - PWM 1
 * Drive Motor 2 (FR) - Jaguar - PWM 2
 * Drive Motor 3 (RL) - Jaguar - PWM 3
 * Drive Motor 4 (RR) - Jaguar - PWM 4

 * Lift Motor - Jaguar - PWM 5
 * Arm Rotation - 2 Jaguar - PWM 6 split

 * Arm Pneumatics - 2 Relays - 
 * Minibot Deployment - Spike - 
 * Compressor - Spike - 
 * Compressor PSI switch - Digitial IO X
*/

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class RobotDemo : public SimpleRobot //DECLARING
{
	RobotDrive *myRobot; // robot drive system
	Joystick *leftstick; // only joystick
	Joystick *rightstick; // only joystick
	Joystick *armstick;
	Relay *k_relay; // only relay
	Compressor *compressor; // Compressor
	Solenoid *soy[2]; //sauce 
	Jaguar *forkliftjag; //jag for forklift
	Jaguar *flexjag;//jag for flexing the grabber arm hand

	int piston_position; // 0 down, 1 up
	//Task *cameraTask;

public:
	RobotDemo(void) //CREATING
	{
		myRobot = new RobotDrive(1, 3, 2, 4);// these must be initialized in the same order
		leftstick = new Joystick(1); // as they are declared above.
		rightstick = new Joystick(2); // as they are declared above.
		armstick = new Joystick(3);
		k_relay = new Relay(2,Relay::kForwardOnly);
		compressor = new Compressor(4,2);
		soy[0]= new Solenoid(1);
		soy[1]= new Solenoid(4);
		piston_position = 0;
		forkliftjag = new Jaguar(5);
		flexjag = new Jaguar(6);

		myRobot->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		myRobot->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);

		myRobot->SetExpiration(0.1);
		SmartDashboard::init();
		SmartDashboard::Log("initializing...", "System State");

		//camera->WriteBrightness(0);

		//cameraTask = new Task("Camerastuff",(FUNCPTR)&Camerastuff);

		Wait(3.0);
	}

	/**
	 * Drive left & right motors for 1 seconds then stop
	 */
	void Autonomous(void) {
		SmartDashboard::Log("Autonomous", "System State");
		myRobot->SetSafetyEnabled(false);
		//myRobot->Drive(0.1, 0.0); 	// drive forwards tenth speed
		Wait(1.0); //    for 1 seconds
		//myRobot->Drive(0.0, 0.0); 	// stop robot


	}

	
	static void Camerastuff() {
		AxisCamera *camera; //Cameralol (: 
		HSLImage *hslimage;
		vector<ParticleAnalysisReport>* pars;
		Threshold tapeThreshold(43, 44, 250, 255, 96, 255);
		BinaryImage *tapePixels;
		ParticleAnalysisReport par;

		camera = &AxisCamera::GetInstance();
		camera->WriteResolution(AxisCameraParams::kResolution_160x120);

		while (1) {

			hslimage = camera->GetImage();

			tapePixels = hslimage->ThresholdHSL(tapeThreshold);
			pars = tapePixels->GetOrderedParticleAnalysisReports();
			if (pars->size() > 0) {
				par = (*pars)[0];
				SmartDashboard::Log(par.center_mass_x, "center of mass x");
				SmartDashboard::Log(par.center_mass_y, "center of mass y");
			}

			if (pars->size() > 0) {
				double closest_x_val = (*pars)[0].center_mass_x_normalized;
				for (unsigned int i=0; i<pars->size(); i++) {
					par = (*pars)[i];
					if (fabs(par.center_mass_x_normalized)
							< fabs(closest_x_val) && par.particleToImagePercent
							> 0.0001) {
						closest_x_val = par.center_mass_x_normalized;
					}
				}
				if (closest_x_val > 0) {
					//we're too far to the left (anti-clockwise)
			//		myRobot->MecanumDrive_Polar(0.5, 0, 0.2);//speed, direction (degrees), rotation (-1..1)
					SmartDashboard::Log("right", "which way");
				} else if (closest_x_val < 0) {
					//too far right (clockwise)
					SmartDashboard::Log("left", "which way");
			//		myRobot->MecanumDrive_Polar(0.5, 0, -0.2);
					
				} else {
					SmartDashboard::Log("straight", "which way");
					//uh, we're already centered
				}
			}
			pars->clear();
			delete pars;

			delete tapePixels;
			delete hslimage;
		}

	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void) {
		SmartDashboard::Log("Teleoperated", "System State");
		myRobot->SetSafetyEnabled(true);

		//cameraTask->Start((UINT32)myRobot);

		while (IsOperatorControl()) {

			//myRobot->TankDrive(leftstick, rightstick);
			myRobot->MecanumDrive_Polar(rightstick->GetMagnitude(),
					rightstick->GetDirectionDegrees(), leftstick->GetX());
			//	myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
			Wait(0.005); // wait for a motor update time

		
			if (armstick->GetRawButton(1)) //Tells us if the trigger is being pressed on the leftstick
			{
				SmartDashboard::Log("Yes", "Trigger Pressed");
				//k_relay->Set(Relay::kOn);

			} else {
				SmartDashboard::Log("No", "Trigger Pressed");
				//k_relay->Set(Relay::kOff);
			}

			if (armstick->GetTop()) //Tells us if the trigger is being pressed on the leftstick
			{
				SmartDashboard::Log("Yes", "hat Pressed");
				//k_relay->Set(Relay::kOn);

			} else {
				SmartDashboard::Log("No", "hat Pressed");
				//k_relay->Set(Relay::kOff);
			}
			
			SmartDashboard::Log( armstick->GetX(), "Armstick X");
			SmartDashboard::Log( armstick->GetY(), "Armstick Y");
			SmartDashboard::Log( armstick->GetZ(), "Armstick Z");
			
			forkliftjag->Set(0);
			flexjag->Set(0);
			
			if(fabs(armstick->GetY()) > 0.5)
			{
				forkliftjag->Set(armstick->GetY()/2.0);
			}
			
			if(fabs(armstick->GetX()) > 0.5)
			{
				flexjag->Set(armstick->GetX()/2.0);
			}

			DriverStationEnhancedIO &controller_box =
					DriverStation::GetInstance()->GetEnhancedIO();
			if (controller_box.GetDigital(3)) {
				compressor->Start();
				SmartDashboard::Log( "ON", "compressor");
				//starts compressor when switch 3 flicked
			} else {
				compressor->Stop();
				SmartDashboard::Log( "OFF", "compressor");
			}

			if (controller_box.GetDigital(4)) //4 is the Solenoid switcherrooo thinggyyy 
			{
				piston_up();
				SmartDashboard::Log( "UP", "piston");
				//makes piston go upp
			} else {
				piston_down();
				SmartDashboard::Log( "DOWN", "piston");
				//makes piston go down down down down down 
			}

		}

	}

	void piston_up(void) {
		if (piston_position == 0) {
			soy[0]->Set(true);
			soy[1]->Set(false);
			Wait(0.03);
			soy[0]->Set(false);
			soy[1]->Set(false);
			piston_position = 1;
		}
	}

	void piston_down(void) {
		if (piston_position == 1) {
			soy[0]->Set(false);
			soy[1]->Set(true);
			Wait(0.03);
			soy[0]->Set(false);
			soy[1]->Set(false);
			piston_position = 0;
		}

	}

};




START_ROBOT_CLASS(RobotDemo)
;

