#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive *myRobot; // robot drive system
	Joystick *leftstick; // only joystick
	Joystick *rightstick; // only joystick
	Relay *k_relay; // only relay
	Compressor *C1; // Compressor

public:
	RobotDemo(void)
	{
		myRobot = new RobotDrive(1, 2, 3, 4);	// these must be initialized in the same order
		leftstick = new Joystick(1);		// as they are declared above.
		rightstick = new Joystick(2);		// as they are declared above.
		k_relay = new Relay(2,Relay::kForwardOnly);
		C1 = new Compressor(4,2);

		myRobot->SetExpiration(0.1);
		SmartDashboard::init();
		SmartDashboard::Log("initializing...", "System State");
		
		AxisCamera &camera = AxisCamera::GetInstance();
		camera.WriteResolution(AxisCameraParams::kResolution_160x120);
		camera.WriteBrightness(0);
		Wait(3.0);
	}

	/**
	 * Drive left & right motors for 1 seconds then stop
	 */
	void Autonomous(void)
	{
		SmartDashboard::Log("Autonomous", "System State");
		myRobot->SetSafetyEnabled(false);
		myRobot->Drive(0.1, 0.0); 	// drive forwards tenth speed
		Wait(1.0); 				//    for 1 seconds
		myRobot->Drive(0.0, 0.0); 	// stop robot
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		SmartDashboard::Log("Teleoperated", "System State");
		myRobot->SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			myRobot->TankDrive(leftstick, rightstick);
		//	myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
			Wait(0.005);				// wait for a motor update time
			
			if (leftstick->GetRawButton(1)) //Tells us if the trigger is being pressed on the leftstick
			{	
				SmartDashboard::Log("Yes", "Trigger Pressed");
				//k_relay->Set(Relay::kOn);
				
			}
			else 
			{
				SmartDashboard::Log("No", "Trigger Pressed");
				//k_relay->Set(Relay::kOff);
			}
			
			DriverStationEnhancedIO& controller_box = (DriverStation::GetInstance()->GetEnhancedIO());
			if (controller_box.GetDigital(3))
				{    
					C1-> Start();	
					//starts compressor when switch 3 flicked
				}
			else
				{
					C1-> Stop();
				}
		}
		
		
	}
};

START_ROBOT_CLASS(RobotDemo);

