/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <iostream>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include <RobotDrive.h>
#include <XboxController.h>
#include <Driverstation.h>
#include <DoubleSolenoid.h>

class Robot : public frc::IterativeRobot {
public:

	frc::DoubleSolenoid *sigmaShift,  *sigmaLift, *sigmaIntake;
	frc::RobotDrive *sigmaDrive, *boostDrive;
 	WPI_TalonSRX * right1 = new WPI_TalonSRX(1);
	WPI_TalonSRX * right2 = new WPI_TalonSRX(2);
	WPI_TalonSRX * right3 = new WPI_TalonSRX(3);
	WPI_TalonSRX * left1 = new WPI_TalonSRX(4);
	WPI_TalonSRX * left2 = new WPI_TalonSRX(5);
	WPI_TalonSRX * left3 = new WPI_TalonSRX(6);
	//sigmaDrive = new robotDrive(right1, left1);
	//boostDrive = new robotDrive(right2, left2);

	XboxController * controller = new XboxController(0);

	//XboxController buttons
	bool X = (controller->GetRawButton(3));
	bool A = (controller->GetRawButton(1));
	bool Y = (controller->GetRawButton(4));
	bool RB = (controller->GetRawButton(6));
	bool LB = (controller->GetRawButton(5));
	bool LS = (controller->GetRawButton(9));
	double LT = (controller->GetRawAxis(2));
	double RT = (controller->GetRawAxis(3));
	double LY = controller->GetY(GenericHID::JoystickHand(0));
	double RY = controller->GetY(GenericHID::JoystickHand(1));

	double leftValue = LY; /*left motor value (TeleopPeriodic)*/
	double rightValue = RY; /*right motor value (TeleopPeriodic)*/

	void RobotInit() {

	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {

		//xbox controller values
		LT = (controller->GetRawAxis(2));
		RT = (controller->GetRawAxis(3));
		LY = controller->GetY(GenericHID::JoystickHand(0));
		RY = controller->GetY(GenericHID::JoystickHand(1));

		leftValue = LY;
		rightValue = RY;

		//drive train values
		left1->Set(leftValue); //add follow command later
		left2->Set(leftValue);
		left3->Set(leftValue);
		
		right1->Set(rightValue); //add follow command later
		right2->Set(rightValue);
		right3->Set(rightValue);

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
