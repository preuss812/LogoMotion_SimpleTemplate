#include "WPILib.h"

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
	Relay *k_relay; // only relay
	Compressor *C1; // Compressor
	Solenoid *soy[2]; //sauce 
	AxisCamera *camera; //Cameralol(: 
	HSLImage *hslimage;
	int piston_position; // 0 down, 1 up
	DigitalInput *digimon; // Digitial Input 
	Encoder *encoder;
	
	

public:
	RobotDemo(void) //CREATING
	{
		myRobot = new RobotDrive(1, 3, 2, 4);// these must be initialized in the same order
		leftstick = new Joystick(1);		// as they are declared above.
		rightstick = new Joystick(2);		// as they are declared above.
		k_relay = new Relay(2,Relay::kForwardOnly);
		C1 = new Compressor(4,2);
		soy[0]= new Solenoid(1);
		soy[1]= new Solenoid(4);
		piston_position = 0;
		digimon = new DigitalInput(4, 12);
		encoder = new Encoder(4,1,4,2,true);
		
		myRobot->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		myRobot->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		
		
		myRobot->SetExpiration(0.1);
		SmartDashboard::init();
		SmartDashboard::Log("initializing...", "System State");
		
		camera = &AxisCamera::GetInstance();
		camera->WriteResolution(AxisCameraParams::kResolution_160x120);
		//camera->WriteBrightness(0);
		Wait(3.0);
	}

	/**
	 * Drive left & right motors for 1 seconds then stop
	 */
	void Autonomous(void)
	{
		SmartDashboard::Log("Autonomous", "System State");
		myRobot->SetSafetyEnabled(false);
		//myRobot->Drive(0.1, 0.0); 	// drive forwards tenth speed
		Wait(1.0); 				//    for 1 seconds
		//myRobot->Drive(0.0, 0.0); 	// stop robot
		

		
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		SmartDashboard::Log("Teleoperated", "System State");
		myRobot->SetSafetyEnabled(true);
		vector<ParticleAnalysisReport>* pars;
		Threshold pinkThreshold(226, 255, 28, 255, 96, 255);
		BinaryImage *pinkPixels;
		ParticleAnalysisReport par; 
		while (IsOperatorControl())
		{
			
			hslimage = camera->GetImage();
			
			
			
			pinkPixels = hslimage->ThresholdHSL(pinkThreshold);
			
			pars = pinkPixels->GetOrderedParticleAnalysisReports();
			if (pars->size() > 0)
			{
				par = (*pars)[0];
				SmartDashboard::Log(par.center_mass_x, "center of mass x");
				SmartDashboard::Log(par.center_mass_y, "center of mass y");
			}
			pars->clear();
			delete pars;
				
			//myRobot->TankDrive(leftstick, rightstick);
			myRobot->MecanumDrive_Polar(rightstick->GetMagnitude(),rightstick->GetDirectionDegrees(),leftstick->GetX());
			//	myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
			Wait(0.005);				// wait for a motor update time
			
			if (leftstick->GetRawButton(1)) //Tells us if the trigger is being pressed on the leftstick
			{	
				//SmartDashboard::Log("Yes", "Trigger Pressed");
				//k_relay->Set(Relay::kOn);
				
			}
			else 
			{
				//SmartDashboard::Log("No", "Trigger Pressed");
				//k_relay->Set(Relay::kOff);
			}
			
			DriverStationEnhancedIO &controller_box = 
				DriverStation::GetInstance()->GetEnhancedIO();
			if (controller_box.GetDigital(3))
				{    
					C1-> Start();	
					//starts compressor when switch 3 flicked
				}
			else
				{
					C1-> Stop();
				}
			
			if (controller_box.GetDigital(4)) //4 is the Solenoid switcherrooo thinggyyy 
							{    
								piston_up();	
								//makes piston go upp
							}
						else
							{
								piston_down();
								//makes piston go down down down down down 
							}
			
			if (digimon->Get() == 1)
				{
					//SmartDashboard::Log("1", "DigitalLight");
				}
			else
				{
					//SmartDashboard::Log("0", "DigitalLight");
				}
			SmartDashboard::Log(encoder->GetRaw(), "EncoderValue");
		}
		
	}

	void piston_up(void)
	{
		if(piston_position == 0)
		{
			soy[0]->Set (true);
			soy[1]->Set (false);
			Wait(0.03);
			soy[0]->Set (false);
			soy[1]->Set (false);
			piston_position = 1;
		}
	}

	void piston_down(void)
	{
		if(piston_position == 1)
		{
			soy[0]->Set (false);
			soy[1]->Set (true);
			Wait(0.03);
			soy[0]->Set (false);
			soy[1]->Set (false);
			piston_position = 0;
		}

	}
	
		
		
	
};

//this is a test


START_ROBOT_CLASS(RobotDemo);

