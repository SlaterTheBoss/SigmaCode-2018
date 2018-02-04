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

#define STATE1 		1
#define STATE1_5 	2
#define STATE2 		3
#define STATE2_5 	4
#define STATE3 		5

class Robot : public frc::IterativeRobot {
public:

	std::string gamedata;
	std::string _sb;
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

	WPI_TalonSRX * rightDrive = new WPI_TalonSRX(1);
	WPI_TalonSRX * right2 = new WPI_TalonSRX(2);
	WPI_TalonSRX * right3 = new WPI_TalonSRX(3);
	WPI_TalonSRX * leftDrive = new WPI_TalonSRX(4);
	WPI_TalonSRX * left2 = new WPI_TalonSRX(5);
	WPI_TalonSRX * left3 = new WPI_TalonSRX(6);
	WPI_VictorSPX *lift1 = new WPI_VictorSPX(7);
	WPI_VictorSPX *lift2 = new WPI_VictorSPX(8);

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
	//system states (for actuators with multiple states)
	bool state_RB = false;
	bool state_LB = false;

	// For Elevator state
	int liftState = STATE1;
	int nextState = STATE1;

	DigitalInput *liftSwitch1 = new DigitalInput(0);
	DigitalInput *liftSwitch2 = new DigitalInput(1);
	DigitalInput *liftSwitch3 = new DigitalInput(2);


	/*
	DigitalInput liftSwitch1(1);
	DigitalInput liftSwitch1(2);
	DigitalInput liftSwitch1(3);
	*/

    double leftDriveValue;
	double rightDriveValue;

	void RobotInit() {

	}

	void AutonomousInit() override {

		std::cout << "Auto Start!";
		gamedata = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		if(gamedata[0] == 'L')
		{
			leftDrive->Set(0.3);
		}
		else
		{
			leftDrive->Set(1.0);
		}

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {
	}

	void TeleopPeriodic() {

		//following motors (slave motors)
		left2->Follow(*leftDrive);
		left3->Follow(*leftDrive);
		right2->Follow(*rightDrive);
		right3->Follow(*rightDrive);
		lift2->Follow(*lift1);
		lift2->SetInverted(true);

		//xbox controller values
		A = (controller->GetRawButtonPressed(1));
		X = (controller->GetRawButtonPressed(3));
		Y = (controller->GetRawButtonPressed(4));
		B = (controller->GetRawButton(2));
		//RB = (controller->GetRawButtonPressed(6));
		RB = (controller->GetRawButton(6));
		LB = (controller->GetRawButtonPressed(5));
		LS = (controller->GetRawButton(9));
		LT = (controller->GetRawAxis(2));
		RT = (controller->GetRawAxis(3));
		LY = controller->GetY(GenericHID::JoystickHand(0));
		RY = controller->GetY(GenericHID::JoystickHand(1));

		leftDriveValue = -LY;
        rightDriveValue = RY;

        //drive train values
		leftDrive->Set(leftDriveValue); //add follow command
		rightDrive->Set(rightDriveValue); //add follow command

		int pulseWidthPosLeft = leftDrive->GetSensorCollection().GetPulseWidthPosition();
		int pulseWidthPosRight = rightDrive->GetSensorCollection().GetPulseWidthPosition();
		printf("L-  ");
		printf("%i", pulseWidthPosLeft);
		printf("R-  ");
		printf("\n");
		printf("%i", pulseWidthPosRight);
		printf("\n");
		printf("\n");
		

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

					float heading_error = tx;
					float steering_adjust = 0.0f;

	    			speed = (-ta/100) + 1;
	               	/*
	    			if (tx > 0.0)
					{
						leftDrive->Set(speed);
					}
					else if (tx < 0.0)
					{
						leftDrive->Set(-speed);
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

	    			leftDrive->Set(leftDrive->Get() + steering_adjust);
	    			rightDrive->Set(rightDrive->Get() + steering_adjust);

				} //sets distance limit using 'ta'
			} //closes 'if target detected' loop
    	} //closes vision function

    	if(liftSwitch1->Get())
    	{
    		printf("Bottom\n");
    	}
    	if(liftSwitch2->Get())
    	{
    		printf("Middle\n");
    	}
    	if(liftSwitch3->Get())
    	{
    		printf("Top\n");
    	}

    	//Elevator code
    	if(A){nextState = STATE1;}
    	if(X){nextState = STATE2;}
    	if(Y){nextState = STATE3;}

    	switch (liftState)
    	{
    	    case STATE1:
    	    	//printf("State1 \n");
				if(nextState == STATE1)
				{
					lift1->Set(0.0);
				}
				else
				{
					lift1->Set(0.4);
					liftState = STATE1_5;
				}
				break;

			case STATE1_5:
				//printf("State1_5 \n");
				if(nextState <= STATE1)
				{
					lift1->Set(-0.4);

					if(liftSwitch1->Get())
					{
						liftState = STATE1;
					}
				}

				if(nextState >= STATE2)
				{
					lift1->Set(0.4);

					if(liftSwitch2->Get())
					{
						liftState = STATE2;
					}
				}
				break;

			case STATE2:
				//printf("State2 \n");
				if(nextState == STATE2)
				{
					lift1->Set(0.0);
				}
				else if(nextState >= STATE3)
				{
					lift1->Set(0.4);
					liftState = STATE2_5;
				}
				else if(nextState <= STATE1)
				{
					lift1->Set(-0.4);
					liftState = STATE1_5;
				}

				break;

			case STATE2_5:
				//printf("State2_5 \n");
				if(nextState <= STATE2)
				{
					lift1->Set(-0.4);

					if(liftSwitch2->Get())
					{
						liftState = STATE2;
					}
				}

				if(nextState >= STATE3)
				{
					lift1->Set(0.4);

					if(liftSwitch3->Get())
					{
						liftState = STATE3;
					}
				}
				break;

			case STATE3:
				//printf("State3 \n");
				if(nextState == STATE3)
				{
					lift1->Set(0.0);
				}

		        if(nextState <= STATE2)
				{
					lift1->Set(-0.4);
					liftState = STATE2_5;
				}
		    	break;
		} //end of elevator code

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
