/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <iostream>
#include <IterativeRobot.h>
#include <Driverstation.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SmartDashboard.h>
#include <SmartDashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
#include <RobotDrive.h>
#include <XboxController.h>
#include <Compressor.h>
#include <DoubleSolenoid.h>
//#include <NetworkTable.h> //try to fix later
#include "WPIlib.h" //replace with network-tables later

class Robot : public frc::IterativeRobot {
public:

	std::string gamedata;

	//limelight network table declarations
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	float tv; //targets detected
	float ta; //target area in '%'
	float tx; //x axis values
	float ty; //y axis values
	//limelight vision logic declarations
	double speed;
	float Kp = -0.1f;
	float min_command = 0.05f;

	Compressor *compressor = new Compressor(0);
	frc::DoubleSolenoid sigmaShift{0, 1}; // Shifts the gears
	frc::DoubleSolenoid sigmaIntake1{2, 3}; // grabs the cube for in-take
	frc::DoubleSolenoid sigmaIntake2{4, 5}; // grabs the cube for in-take
	frc::DoubleSolenoid sigmaSucc{6, 7}; //

	WPI_TalonSRX * right1 = new WPI_TalonSRX(1);
	WPI_TalonSRX * right2 = new WPI_TalonSRX(2);
	WPI_TalonSRX * right3 = new WPI_TalonSRX(3);
	WPI_TalonSRX * left1 = new WPI_TalonSRX(4);
	WPI_TalonSRX * left2 = new WPI_TalonSRX(5);
	WPI_TalonSRX * left3 = new WPI_TalonSRX(6);

	//XboxController
	XboxController * controller = new XboxController(0);
	bool X;
	bool A;
	bool Y;
	bool B;
	bool RB;
	bool LB;
	bool LS;
	double LT;
	double RT;
	double LY;
	double RY;
	//button states (for actuators with multiple states)
	bool state_RB = false;
	bool state_LB = false;

	double leftValue; /*left motor value (TeleopPeriodic)*/
	double rightValue; /*right motor value (TeleopPeriodic)*/

	void RobotInit() {

	}

	void AutonomousInit() override {

		std::cout << "Auto Start!";
		gamedata = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		if(gamedata[0] == 'L')
		{
			left1->Set(0.3);
		}
		else
		{
			left1->Set(1.0);
		}

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {
	}

	void TeleopPeriodic() {

		//xbox controller values
		X = (controller->GetRawButton(3));
		A = (controller->GetRawButton(1));
		Y = (controller->GetRawButton(4));
		B = (controller->GetRawButton(2));
		//RB = (controller->GetRawButtonPressed(6));
		RB = (controller->GetRawButton(6));
		LB = (controller->GetRawButtonPressed(5));
		LS = (controller->GetRawButton(9));
		LT = (controller->GetRawAxis(2));
		RT = (controller->GetRawAxis(3));
		LY = controller->GetY(GenericHID::JoystickHand(0));
		RY = controller->GetY(GenericHID::JoystickHand(1));

		leftValue = LY;
		rightValue = -1 * RY;

		//drive train values
		left1->Set(leftValue); //add follow command
		left2->Set(leftValue); //removed for testing **WORKING***
		left3->Set(leftValue);
		right1->Set(rightValue); //add follow command
		right2->Set(rightValue);
		right3->Set(rightValue);

		//limelight vision code
		tx = table->GetNumber("tx", 0.0);
		ty = table->GetNumber("ty", 0.0);
		tv = table->GetNumber("tv", 0.0);
		ta = table->GetNumber("ta", 0.0);

		if (B)
		{
			/*
			 * if the target is found: robot turns and moves towards the object
			 * if target area is greater than 85%: robot will stop
			 * if no target is found: do nothing
			 */

			if(tv == 1) //If a target is detected
			{

				if(ta < 75)
				{

					float heading_error = -tx;
					float steering_adjust = 0.0f;

					speed = (-ta/100) + 1;
					/*
					if (tx > 0.0)
					{
						left1->Set(speed);
					}
					else if (tx < 0.0)
					{
						left1->Set(-speed);
					}
					*/

					//motor speed does not go over 1
					if (tx > 0.0)
					{
						steering_adjust = Kp * heading_error - min_command;
					}
					else if (tx < 0.0)
					{
						steering_adjust = Kp * heading_error + min_command;
					}

					left1->Set(left1->Get() + steering_adjust);
					left2->Set(left2->Get() + steering_adjust);
					left3->Set(left3->Get() + steering_adjust);
					right1->Set(right1->Get() - steering_adjust);
					right2->Set(right2->Get() - steering_adjust);
					right3->Set(right3->Get() - steering_adjust);

				} //sets distance limit using 'ta'
			} //closes 'if target detected' loop
		} //loses vision function

		//Pneumatics Code
		compressor->SetClosedLoopControl(true);

/*
		if(RB && (state_RB == false))
		{
			sigmaGift.Set(frc::DoubleSolenoid::Value::kForward);
			state_RB = !state_RB;
		}
		else if(RB && (state_RB == true))
		{
			sigmaGift.Set(frc::DoubleSolenoid::Value::kReverse);
			state_RB = !state_RB;
		}
*/

		//
		if(RB == true){
			sigmaShift.Set(frc::DoubleSolenoid::Value::kForward);

		}
		else{
			sigmaShift.Set(frc::DoubleSolenoid::Value::kReverse);
		}


	}
	void TestPeriodic() {

	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;

};

START_ROBOT_CLASS(Robot)
