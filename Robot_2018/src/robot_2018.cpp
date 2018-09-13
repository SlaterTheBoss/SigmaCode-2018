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
#include <ADXRS450_Gyro.h>
//#include <NetworkTable.h> //try to fix later
#include "WPIlib.h" //replace with network-tables

#define TICKS_PER_INCH 1672
/*
#define STATE1 		1
#define STATE1_5 	2
#define STATE2 		3
#define STATE2_5 	4
#define STATE3 		5
*/
class Robot : public frc::IterativeRobot {
public:

	std::string gamedata;

//	std::string _sb;
//	limelight network table declarations

	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");

	float tv; //targets detected
	float ta; //target area in '%'
	float tx; //x axis values
	float ty; //y axis values

	//limelight vision logic declarations
	double speed;
	float Kp = -0.025f; //-0.037 casts 'tx' directly to a value of 0 to 1
	float min_command = 0.05f;
	int tx_Sign = 0;
	int tx_Val = 0;

	int pulseWidthPosLeft = 0;
	int pulseWidthPosRight = 0;

	Compressor *compressor = new Compressor(0);
	frc::DoubleSolenoid sigmaShift{0, 1}; // Shifts to high gear - LT Press-and-Hold
	frc::DoubleSolenoid sigmaLift{2, 3}; // Elevate - ASCEND! - X Toggle
	frc::DoubleSolenoid sigmaIntake1{4, 5}; // Cube GRIPP! - RT Toggle

	//1,2,3
	WPI_TalonSRX * leftDrive = new WPI_TalonSRX(1);
	WPI_VictorSPX * left2 = new WPI_VictorSPX(2);
	WPI_VictorSPX * left3 = new WPI_VictorSPX(3);

	//4,5,6
	WPI_TalonSRX * rightDrive = new WPI_TalonSRX(4);
	WPI_VictorSPX * right2 = new WPI_VictorSPX(5);
	WPI_VictorSPX * right3 = new WPI_VictorSPX(6);

	WPI_TalonSRX *intakePivot = new WPI_TalonSRX(8);

	WPI_TalonSRX *firstElev1 = new WPI_TalonSRX(7);
	WPI_TalonSRX *secondElev = new WPI_TalonSRX(9);
	WPI_VictorSPX *firstElev2 = new WPI_VictorSPX(10);

	WPI_VictorSPX *intake1 = new WPI_VictorSPX(11);
	WPI_VictorSPX *intake2 = new WPI_VictorSPX(12); // follows intake1

	frc::ADXRS450_Gyro * Gyro = new ADXRS450_Gyro();
	int resetGyro = 0;

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
	int DP;
	bool stateX = false;
	bool stateRT = false;


	XboxController * controller2 = new XboxController(1);
	bool Y2;
	bool A2;

	bool intakeState = 0;
	bool intakeStateFlipped = 0;

	double liftEncoder1;
	double liftEncoder2;

	double liftEncoderMax1;
	double liftEncoderMin1;
	double liftEncoderMax2;
	double liftEncoderMin2;

	//int initEncoder = 0;

    double leftDriveValue;
	double rightDriveValue;

	double rightOffset = 0.95;

	int rInvert = 0;
	int cInvert = 0;
	double cAngle = 0;
	int cDistance = 0;
	int autoState = 0;
	int moveState = 0;
	int baseCurrent;
	int targetPulseWidthPosRight = 0;


	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(autoLine, autoLine);
		m_chooser.AddObject(leftAuto, leftAuto);
		m_chooser.AddObject(centerAuto, centerAuto);
		m_chooser.AddObject(rightAuto, rightAuto);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		Gyro->Calibrate();
	}

	void sigmaDrive(double left, double right) {
		leftDrive->Set(left);
		rightDrive->Set(right * rightOffset);
	}

	bool moveStraight(int inches, double speed) {
		bool ret = 0;

		//printf("%i as \n", moveState);

		switch (moveState)
		{
			case 0 :
				pulseWidthPosRight = rightDrive->GetSensorCollection().GetPulseWidthPosition();
				targetPulseWidthPosRight = pulseWidthPosRight + (inches * TICKS_PER_INCH);
				//printf("%i    %i\n\n", pulseWidthPosRight, targetPulseWidthPosRight);
				moveState = 1;
				break;

			case 1 :
				sigmaDrive(-speed, speed);
				moveState = 2;
				break;

			case 2 :
				if (18000 < pulseWidthPosRight && pulseWidthPosRight < 21000)
				{
					baseCurrent = leftDrive->GetOutputCurrent();
					printf("%i as \n", baseCurrent);
				}

				if (pulseWidthPosRight < targetPulseWidthPosRight)
				{
					pulseWidthPosRight = rightDrive->GetSensorCollection().GetPulseWidthPosition();
					//printf("%i   %i\n\n", pulseWidthPosRight, targetPulseWidthPosRight);
				}
				else
				{
					sigmaDrive(0, 0);
					moveState = 0;
					ret = true;
				}

				break;
		}

		return ret;
	}

	bool GyroTurn(double angle) {

		bool ret = 0;

		//printf("%i rg \n", resetGyro);

		switch (resetGyro) {

			case 0 :
				Gyro->Reset();
				resetGyro = 1;
				break;

			case 1 :
				double GyroAngle = Gyro->GetAngle();

				printf("%f  \n", GyroAngle);

				if((angle - 1.0) < GyroAngle && GyroAngle < (angle + 1.0)) // When Gyro Angle reaches the range it stops
				{
					sigmaDrive(0.0, 0.0);
					resetGyro = 0;
					ret = 1;
				}
				else if(angle > GyroAngle) // When GyroAngle is Less it turns right
				{
					printf("R  ");
					sigmaDrive(-0.5, -0.5);
				}
				else if(angle < GyroAngle) // When GyroAngle is Greater it turns left
				{
					printf("L  ");
					sigmaDrive(0.5, 0.5);
				}
				break;
		}

		return ret;
	}

        ////////////////////////////////////////////////////////
    ////////Try to put into separate class/header file later////////
        ////////////////////////////////////////////////////////
	void shortSideAuto(){
		bool status = 0;
		switch (autoState)
		{
			case 0 :
				status = moveStraight(140, 0.65);
				if(status == 1)
				{
					autoState = 1;
				}
				break;

			case 1 :
				status = GyroTurn(rInvert * 70);
				if(status == 1)
				{
					autoState = 2;
				}
				break;

			case 2 :
				status = moveStraight(17, 0.65);
				sigmaLift.Set(frc::DoubleSolenoid::Value::kReverse);

				if(status == 1)
				{
					autoState = 3;
				}
				break;

			case 3 :
				intake1->Set(0.5);
				intake2->Set(-0.5);
				break;
		}
    }

	void centerAutoScore(){
		bool status = 0;

		switch (autoState)
		{
			case 0 :
				sigmaIntake1.Set(frc::DoubleSolenoid::Value::kReverse);

				status = moveStraight(24, 0.65);
				if(status == 1)
				{
					autoState = 1;
				}
				break;

			case 1 :
				status = GyroTurn(cInvert * cAngle);
				if (status == 1)
				{
					autoState = 2;
				}
				break;

			case 2 :
				status = moveStraight(cDistance, 0.65);
				if (status == 1)
				{
					autoState = 3;
				}
				sigmaLift.Set(frc::DoubleSolenoid::Value::kReverse);
				break;

			case 3 :

				if (gamedata[0] == 'R')
				{
					sigmaDrive(0.0, 0.25);
				}
				else if (gamedata[0] == 'L')
				{
					sigmaDrive(-0.25, 0.0);
				}

				Wait(0.50);
				sigmaDrive(0.0, 0.0);
			//	Wait(1.0);

				//Wait(1.5);
				intake1->Set(0.5);
				intake2->Set(-0.5);
				Wait(0.3);
				sigmaIntake1.Set(frc::DoubleSolenoid::Value::kForward);

				break;
		}
	}

	void baseLineAuto(){
		bool status = 0;

		switch (autoState)
		{
			case 0 :
				status = moveStraight(120, 0.65);

				if(status == 1)
				{
					autoState = 1;
				}
				break;
		}
	}

	void AutonomousInit() override {
		std::cout << "Auto Start!";

		m_autoSelected = m_chooser.GetSelected();
		gamedata = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		if(m_autoSelected == leftAuto){rInvert = 1;}
		else if(m_autoSelected == rightAuto){rInvert = -1;}

		if(gamedata[0] == 'L'){cInvert = -1; cAngle = 35.0; cDistance = 90;}
		else if(gamedata[0] == 'R'){cInvert = 1; cAngle = 23.5; cDistance = 90;}

		autoState = 0;
	}

	void AutonomousPeriodic() {
	//	printf("%i rs \n", autoState);

		if(m_autoSelected == autoLine)
		{
			baseLineAuto();
		}
		else if(m_autoSelected == leftAuto)
		{
			if (gamedata[0] == 'L')
			{
				shortSideAuto();
			}
			else if (gamedata[0] == 'R')
			{
				baseLineAuto();
			}
		}
		else if(m_autoSelected == rightAuto)
		{
			if (gamedata[0] == 'R')
			{
				shortSideAuto();
			}
			else if (gamedata[0] == 'L')
			{
				baseLineAuto();
			}
		}
		else if(m_autoSelected == centerAuto)
		{
			centerAutoScore();
		}
	}

	void TeleopInit() {
		rightDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
		leftDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);

		intake1->Set(0.0);
		intake2->Set(0.0);

		sigmaDrive(0.0, 0.0);

		// second elev stage
		firstElev2->SetInverted(true);
		firstElev2->Follow(*firstElev1);

		//	following motors (slave motors)
		left2->Follow(*leftDrive); //drive train L2
		left3->Follow(*leftDrive); //drive train L3
		right2->Follow(*rightDrive); //drive train R2
		right3->Follow(*rightDrive); //drive train R3

	//	intake2->Follow(*intake1); //intakeMotor 2

	//	intake2->SetInverted(true);

	//	firstLift2->Follow(*firstLift); //elevator 1
	//	firstLift2->SetInverted(true);

		liftEncoderMax1 = 0;
		liftEncoderMin1 = 0;
		liftEncoderMax2 = 0;
		liftEncoderMin2 = 0;
	}

	void processMotors()
	{
		SmartDashboard::PutNumber("Left Joystick", leftDriveValue);
		SmartDashboard::PutNumber("Right Joystick", rightDriveValue);

		//////////////////////////////////////
		/// drivetrain motors
		/////////////////////////////////////
		sigmaDrive(leftDriveValue, rightDriveValue);

		//////////////////////////////////////
		/// intake motors
		/////////////////////////////////////
		if (LB) // pull in box
		{
			intake1->Set(-1.0);
			intake2->Set(1.0);
		}
		else if (RB) // spit out box
		{
			intake1->Set(1.0);
			intake2->Set(-1.0);
		}
		else //idle box
		{
			intake1->Set(-0.20);
			intake2->Set(0.20);
		}


		//////////////////////////////////////
		/// intake lift motors
		/////////////////////////////////////
//		int pivotEncoder = intakePivot->GetSensorCollection().GetPulseWidthPosition();
//		printf("Pivot = %d \n", pivotEncoder);

		if (A || LB) // down
		{
			intakePivot->Set(0.50);
		}
		else if (Y) // up
		{
			intakePivot->Set(-0.40);
		}
		else
		{
			intakePivot->Set(-0.13);
		}

		//////////////////////////////////////
		/// elevator motors
		/////////////////////////////////////
//		if (DP == 0)
		if (Y2 == 1)
		{
			liftAscend();
		}
//		else if (DP == 180)
		else if (A2 == 1)
		{
			liftDescend();
		}
		else
		{
			secondElev->Set(-0.07);
			firstElev1->Set(0.04);
		}
	}

	void liftDescend() {

		int firstEncoder1 = firstElev1->GetSensorCollection().GetPulseWidthPosition();
		int secondEncoder1 = secondElev->GetSensorCollection().GetPulseWidthPosition();

		//printf("%d      %d\n", firstEncoder1, secondEncoder1);
		printf("%i" , secondEncoder1);

		if(firstEncoder1 < -1000) //9600 // more neg, goes up
		{
			firstElev1->Set(-0.45);
		}
		else if(firstEncoder1 < 2100) //9600 // more neg, goes up
		{
			firstElev1->Set(-0.15);
		}
		else
		{
			firstElev1->Set(0.04);
		}


		if(secondEncoder1 > 7500) // more pos, goes up
		{
			secondElev->Set(0.005);
		}
		else if(secondEncoder1 > 4000) // more pos, goes up
		{
			secondElev->Set(0.000);
		}
		else
		{
			secondElev->Set(-0.07);
		}
	}

	void liftAscend() {

		int firstEncoder1 = firstElev1->GetSensorCollection().GetPulseWidthPosition();
		int secondEncoder1 = secondElev->GetSensorCollection().GetPulseWidthPosition();
		printf("%i" , secondEncoder1);

		//printf("%d    %d\n", firstEncoder1, secondEncoder1);

		if (firstEncoder1 > -25000) // more neg, goes up
		{
			firstElev1->Set(0.90);
		}
		else
		{
			firstElev1->Set(0.04);
		}

		if(secondEncoder1 < 26000) // more pos, goes up
		{
			secondElev->Set(-0.90);
		}
		else
		{
			secondElev->Set(-0.07);
		}
	}

	void processPneumatics()
	{
		//////////////////////////////////////
		/// elevator
		/////////////////////////////////////
		if (LS)
		{
			if (sigmaLift.Get() == frc::DoubleSolenoid::Value::kForward)
			{
				sigmaLift.Set(frc::DoubleSolenoid::Value::kReverse);
			}
			else
			{
				sigmaLift.Set(frc::DoubleSolenoid::Value::kForward);
			}
		}

		//////////////////////////////////////
		/// drive train shift
		/////////////////////////////////////
    	if(LT > 0.5) // Shifts gears
    	{
    		sigmaShift.Set(frc::DoubleSolenoid::Value::kForward);
    	}
    	else
    	{
    		sigmaShift.Set(frc::DoubleSolenoid::Value::kReverse);
    	}

		//////////////////////////////////////
		/// Intake piston
		/////////////////////////////////////
    	if (RT == 0.0 && !intakeStateFlipped)
    	{
    		intakeState = !intakeState;
    		intakeStateFlipped = 1;
    	}
    	if (RT == 1.0 && intakeState == 1)
		{
			sigmaIntake1.Set(frc::DoubleSolenoid::Value::kReverse);
			intakeStateFlipped = 0;
		}
		else if (RT == 1.0 && intakeState == 0)
		{
			sigmaIntake1.Set(frc::DoubleSolenoid::Value::kForward);
			intakeStateFlipped = 0;
		}
	}

	void TeleopPeriodic() {

		liftEncoder1 = 0;
		liftEncoder2 = 0;

		//xbox controller values
		A = (controller->GetRawButton(1));
		X = (controller->GetRawButtonPressed(3));
		Y = (controller->GetRawButton(4));
		B = (controller->GetRawButton(2));
	//	RB = (controller->GetRawButton(6));
		RB = (controller->GetRawButton(6));
		LB = (controller->GetRawButton(5));
		LS = (controller->GetRawButtonPressed(9));
		LT = (controller->GetRawAxis(2));
		RT = (controller->GetRawAxis(3));
		LY = controller->GetY(GenericHID::JoystickHand(0));
		RY = controller->GetY(GenericHID::JoystickHand(1));
		DP = controller->GetPOV(0);


		Y2 = (controller2->GetRawButton(4));
		A2 = (controller2->GetRawButton(1));


		leftDriveValue = LY; //switch back later
        rightDriveValue = -RY;
		pulseWidthPosLeft = leftDrive->GetSensorCollection().GetPulseWidthPosition();

		//printf("%i" ,pulseWidthPosLeft);

		processMotors();
        processPneumatics();
		processMotors();
	}

	void TestPeriodic() {

	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "!Do Nothing!";
	const std::string autoLine = "AutoLine";
	const std::string leftAuto = "Left Auto";
	const std::string rightAuto = "Right Auto";
	const std::string centerAuto = "Center Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
