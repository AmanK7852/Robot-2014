#include "WPILib.h"
/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
#define STOP_VALUE -0.008 //the motors are calibrated so that -0.008 actually corresponds to 0. Who knows why.
#define ZERO 0 //In a few years, ZERO will have a value of 1. Inflation d-(^_^)z

class  RobotDemo : public SimpleRobot
{
	Joystick* Jdriver;
	Joystick* Joperator;
	Victor* mLeft;
	Victor* mRight;
	Victor* shooterMotor;
	Victor* arm;
    Victor* roller;
    Compressor* comp;
    AnalogChannel* angle;
    DoubleSolenoid* shooter;
    DoubleSolenoid* rollerPiston;
    DigitalInput* shooterLimit;
    DigitalInput* autoSwitch;
	bool isSet;
	bool isRolling;
	bool isOpen;
	bool isFired;
	bool isActivated;
	bool autoLoop;
	bool isReversed;
	bool isGoingToStartingPosition;
	bool inAuto;
	bool inTeleop;
	
    //Finds the angle in degrees based off of a potentiometer voltage.
	double findAngle(double potValue)
	{  
		return (54.584 * (angle->GetVoltage()) - 77.922);
	}
	//Reverse of above operation
	double findPotValue(double angle)
	{
		return ((angle + 77.922) / 54.584);
	}

    
    
    //moves arm to an angle of choice
    void moveArmToAngle(double angleWanted, Victor* arm, AnalogChannel* angle, bool Autonomous, bool Teleop)
    {
        	if(Autonomous){
        	   	while (angle->GetVoltage() > (findPotValue(angleWanted) + 0.06) || angle->GetVoltage() < (findPotValue(angleWanted) - 0.06))
        	   	{
            		if(angle->GetVoltage() < (findPotValue(angleWanted) + 0.06) && angle->GetVoltage() > (findPotValue(angleWanted) - 0.06)) 
      	    		{
      	    			arm->Set(STOP_VALUE);
    	    		}	 
    	    		else 
    	    		{
    	    			if(angle->GetVoltage() > (findPotValue(angleWanted) + 0.06))
        		    		arm->Set(-0.6);
        		    	else if (angle->GetVoltage() < (findPotValue(angleWanted) - 0.06))
        		    		arm->Set(0.6);
        		    	}
        		    	
        	   	}
        	} else if(Teleop){
        		if(angle->GetVoltage() < (findPotValue(angleWanted) + 0.04) && angle->GetVoltage() > (findPotValue(angleWanted) - 0.04)) 
        		{
        			arm->Set(STOP_VALUE);
        		}	 
        		else 
        		{
        			if(angle->GetVoltage() > (findPotValue(angleWanted) + 0.2))
        				arm->Set(-0.6);
        			else if (angle->GetVoltage() < (findPotValue(angleWanted) - 0.2))
        				arm->Set(0.6);
        			else if (angle->GetVoltage() > (findPotValue(angleWanted)+ 0.04))
        				arm->Set(-0.3);
        			else if (angle->GetVoltage() < (findPotValue(angleWanted) - 0.04))
        				arm->Set(0.3);
        		}
        	}
        }
    
public:    
	
	RobotDemo(void)
	{	
		Jdriver = new Joystick(1);				// driver controller
		Joperator = new Joystick(2);			// operator controller
	    mLeft = new Victor(1);					// left wheels
	    mRight = new Victor(2);					// right wheels
		shooterMotor = new Victor(3); 			// motor that cotrols the position of the firing piston
		arm = new Victor(4); 					// motor that controls the angle of the shooting arm
		roller = new Victor(5); 				// motor that brings the ball into claw
		comp = new Compressor(1, 1); 			// air compressor
		angle = new AnalogChannel(3); 			// potentiometer
	    shooter = new DoubleSolenoid(3,4);		// releases the firing piston
	    rollerPiston = new DoubleSolenoid(1,2); // piston which controls the position of the roller
	    shooterLimit = new DigitalInput(2);		// limit switch for shooter plunger
	    autoSwitch = new DigitalInput(3);		// controls 1-ball vs 2-ball autonomous  
	    
	    //Init vars
	    isSet = false;
	    isRolling = false;
	    isOpen = false;
	    isFired = false;
	    autoLoop = true;
	    isReversed = false;
	    isGoingToStartingPosition = false;
	    inAuto = true;
	    inTeleop = true;
	    	    
	    comp->Start();							// start compresser
	}
	void Autonomous(void)
	{		
		if (!autoSwitch->Get())
		{
			moveArmToAngle(40.5, arm, angle, inAuto, !inTeleop);	
			arm->Set(STOP_VALUE);
			rollerPiston->Set(DoubleSolenoid::kReverse); 	//disengage rollers
			Wait(0.5);
			shooter->Set(DoubleSolenoid::kReverse); 		//fire ball
			Wait(0.25);
			rollerPiston->Set(DoubleSolenoid::kForward); 	//reengage rollers
			Wait(0.25);
			shooter->Set(DoubleSolenoid::kForward); 		//reset shooter mechanism
			mRight->Set(-0.90);
			mLeft->Set(0.90);
			Wait(1.5);
			mRight->Set(ZERO);								//stop wheels
			mLeft->Set(ZERO);
			
				
		}
		else		
		{		
			moveArmToAngle(40.5, arm, angle, inAuto, !inTeleop);
			arm->Set(STOP_VALUE);
			Wait(0.25);
			rollerPiston->Set(DoubleSolenoid::kReverse); 	//disengage rollers
			Wait(0.5);
			shooter->Set(DoubleSolenoid::kReverse); 		//fire ball
			Wait(0.25);
			shooter->Set(DoubleSolenoid::kForward); 		//reset shooter mechanism
			rollerPiston->Set(DoubleSolenoid::kForward); 	//reengage rollers
							
			roller->Set(1);									//start rollers
			
			while (angle->GetVoltage() > findPotValue(-27) + 0.06 || angle->GetVoltage() < findPotValue(-27) - 0.06)
			{
				moveArmToAngle(-27, arm, angle, !inAuto, inTeleop); //EXCEPTION, BECAUSE WE HAVE TO DO MULTIPLE ACTIONS SIMULTANEOUSLY
				if(shooterLimit->Get()) // IF SHOOTER LIMIT SWITCH IS PRESSED, STOP PULLING THE PISTON BACK
					shooterMotor->Set(STOP_VALUE);
				else
					shooterMotor->Set(1.0);   
			}  
			shooterMotor->Set(STOP_VALUE);
			arm->Set(STOP_VALUE);

			while (!shooterLimit->Get())
			{
				if(shooterLimit->Get()) // IF SHOOTER LIMIT SWITCH IS PRESSED, STOP PULLING THE PISTON BACK
					shooterMotor->Set(STOP_VALUE);
				else
					shooterMotor->Set(1.0);
			}
			shooterMotor->Set(STOP_VALUE);
			moveArmToAngle(40.5, arm, angle, inAuto, !inTeleop);
			arm->Set(STOP_VALUE);
			roller->Set(STOP_VALUE);
			
			rollerPiston->Set(DoubleSolenoid::kReverse); 	//disengage rollers
			Wait(0.5);
			shooter->Set(DoubleSolenoid::kReverse); 		//fire ball
			Wait(0.25);
			mRight->Set(-1);								//drive forward
			mLeft->Set(1);
			Wait(1.5);
			mRight->Set(ZERO);								//stop wheels
			mLeft->Set(ZERO);
		}
		
		
		
	} //this bracket corresponds with the initial AutonomousControl(){ bracket.
//--------------------------------------------------OPERATOR CONTROL BEGINS HERE----------------------------------------------
	void OperatorControl(void)
	{
		printf("OPERATOR CONTROL");
		SmartDashboard::init();
		mLeft->Set(0);
		mRight->Set(0);
		arm->Set(STOP_VALUE);
		while (IsOperatorControl())
		{
//--------------------------Smart Dashboard Section------------------------------------------------------------------
		SmartDashboard::PutNumber("Potentiometervolt",angle->GetVoltage());
		SmartDashboard::PutBoolean("Limit Switch", shooterLimit->Get());
		SmartDashboard::PutNumber("Driver Axis 5 value", Jdriver->GetRawAxis(5));
		SmartDashboard::PutNumber("Driver Axis 2 value", Jdriver->GetRawAxis(2));
		SmartDashboard::PutNumber("Operator Axis 4 value", Joperator->GetRawAxis(4));
		SmartDashboard::PutNumber("Operator Axis 2 (left Y) value", Joperator->GetRawAxis(2));
		SmartDashboard::PutNumber("Operator Axis 3 value", Joperator->GetRawAxis(3));
		SmartDashboard::PutNumber("Operator Axis 5 (right Y) value", Joperator->GetRawAxis(5));			
		SmartDashboard::PutNumber("Operator Axis 6 (DPad)", Joperator->GetRawAxis(6));
		SmartDashboard::PutBoolean("Operator button 1", Joperator->GetRawButton(1));
		SmartDashboard::PutBoolean("Operator button 2", Joperator->GetRawButton(2));
		SmartDashboard::PutBoolean("Operator button 3", Joperator->GetRawButton(3));
		SmartDashboard::PutBoolean("Operator button 4", Joperator->GetRawButton(4));
		SmartDashboard::PutBoolean("Operator button 5", Joperator->GetRawButton(5));
		SmartDashboard::PutBoolean("Operator button 6", Joperator->GetRawButton(6));
		SmartDashboard::PutBoolean("Operator button 7", Joperator->GetRawButton(7));
		SmartDashboard::PutBoolean("Operator button 8", Joperator->GetRawButton(8));
		SmartDashboard::PutBoolean("Operator button 9", Joperator->GetRawButton(9));
		SmartDashboard::PutBoolean("Operator button 10", Joperator->GetRawButton(10));
		SmartDashboard::PutNumber("Angle", findAngle(angle->GetVoltage()));
		SmartDashboard::PutBoolean("Half Speed", Jdriver->GetRawAxis(3)< -0.3);
		SmartDashboard::PutBoolean("Roller Activated", roller->Get() > 0.2);
		SmartDashboard::PutNumber("Left Motor Power", mLeft->Get());
		SmartDashboard::PutNumber("Right Motor Power", mRight->Get());
		SmartDashboard::PutBoolean("Auto Switch", !autoSwitch->Get());

//--------------------------Driver Joystick Section------------------------------------------------------------------------

			if(Jdriver->GetRawAxis(3) > 0.5){ //left trigger held down
				if(Jdriver->GetRawAxis(4) > 0.2 || Jdriver->GetRawAxis(4) < -0.2){ //cheesy drive
					mLeft->Set(-Jdriver->GetRawAxis(2) + 0.5 * Jdriver->GetRawAxis(4));
					mRight->Set(Jdriver->GetRawAxis(2) + 0.5 * Jdriver->GetRawAxis(4));
				} else {
					mLeft->Set(-0.5* Jdriver->GetRawAxis(2)); //half speed
					mRight->Set(0.5 * Jdriver->GetRawAxis(2));
				}
			} else {
				if(Jdriver->GetRawAxis(3) < -0.3){ // right trigger held down
					mLeft->Set(-Jdriver->GetRawAxis(2) * 0.3);//half speed
					mRight->Set(Jdriver->GetRawAxis(5) * 0.3);//on drive train
				} else if(Jdriver->GetRawAxis(2) < 0.1 && Jdriver->GetRawAxis(2) > -0.1
				&& Jdriver->GetRawAxis(5) < 0.1 && Jdriver->GetRawAxis(5) > -0.1){
					mLeft->Set(-Jdriver->GetRawAxis(2) / 10);
					mRight->Set(Jdriver->GetRawAxis(5) / 10);
				} else if(Jdriver->GetRawAxis(2) < 0.4 && Jdriver->GetRawAxis(2) > -0.4
				&& Jdriver->GetRawAxis(5) < 0.4 && Jdriver->GetRawAxis(5) > -0.4){
					mLeft->Set(-Jdriver->GetRawAxis(2) / 2);
					mRight->Set(Jdriver->GetRawAxis(5) / 2);
				} else {		
					mLeft->Set(-Jdriver->GetRawAxis(2));
					mRight->Set(Jdriver->GetRawAxis(5));
				}
			}
			
			//B button to stop drive train.
			if(Jdriver->GetRawButton(2)){
				mLeft->Set(ZERO);
				mRight->Set(ZERO);
			}
			
			
//------------------------Operator Joystick Section-------------------------------------------------------------			
			
			
			//======================Buttons===================//
			
			//BUTTON A MOVES ARM TO PICKUP POSITION, PULLS PISTON BACK, AND TURNS ROLLERS ON (SIMULTANEOUSLY)
			if(Joperator->GetRawButton(1))
			{
				moveArmToAngle(-27, arm, angle, !inAuto, inTeleop);
		
				if(shooterLimit->Get()) // IF SHOOTER LIMIT SWITCH IS PRESSED, STOP PULLING THE PISTON BACK
					shooterMotor->Set(STOP_VALUE);
				else
					shooterMotor->Set(1.0);
								
				rollerPiston->Set(DoubleSolenoid::kForward);
				roller->Set(1);
				isSet = true;
			} 
			else if (isSet)
			{
				roller->Set(STOP_VALUE);
				shooterMotor->Set(STOP_VALUE);
				isSet = false;
			}
			
			//BUTTON B STOPS ARM MOVEMENT AND SHOOTER MOVEMENT AND ROLLER MOVEMENT
			if(Joperator->GetRawButton(2))
			{
				arm->Set(STOP_VALUE);
				roller->Set(STOP_VALUE);
				shooterMotor->Set(STOP_VALUE);
			}
			
			//BUTTON X PULLS SHOOTER BACK AND ACTIVATES ROLLER
			if (Joperator->GetRawButton(3))
			{	
				if(shooterLimit->Get())
					shooterMotor->Set(STOP_VALUE);
				else
					shooterMotor->Set(1.0);
				
				roller->Set(1);
				isOpen = true;
				
			} else if(isOpen) {
				roller->Set(STOP_VALUE);
				isOpen = false;
				shooterMotor->Set(STOP_VALUE);
			}
			
			//BUTTON Y SPINS ROLLERS OUTWARDS (I.E. RELEASES BALL)
			if(Joperator->GetRawButton(4))
			{
				roller->Set(-1);
				isGoingToStartingPosition = true;
			} else if(isGoingToStartingPosition){
				roller->Set(STOP_VALUE);
				isGoingToStartingPosition = false;
			}
			
			//LEFT BUMPER MOVES ARM TO 37 DEGREES FORWARD (SHOOTING POSITION)
			if(Joperator->GetRawButton(5))
			{
				moveArmToAngle(40, arm, angle, !inAuto, inTeleop);				
			}
			
			//RIGHT BUMPER MOVES ARM TO 25 DEGREES BACKWARD
			//CURRENTLY NOT IN USE
			/*
			if (Joperator->GetRawButton(6))
			{
				moveArmToAngle(135, arm, angle, !inAuto, inTeleop);
			}
			*/
			
			//CLICK BOTH JOYSTICKS TOGETHER TO REVERSE SHOOTER
			if (Joperator->GetRawButton(9) && Joperator->GetRawButton(10))
			{
				shooterMotor->Set(-1);
				isReversed = true;
			} else if (isReversed)
			{
				shooterMotor->Set(STOP_VALUE);
				isReversed = false;
			}
			
			//======================Axes===================//
			
			//LEFT JOYSTICK Y-AXIS HAS MANUAL CONTROL OF ARM MOTOR.
			if(Joperator->GetRawAxis(2) > 0.1 || Joperator->GetRawAxis(2) < -0.1)
			{
				arm->Set(0.40 * Joperator->GetRawAxis(2));
			}
			
			//LEFT TRIGGER SHIFTS ROLLER OUT OF THE WAY SO THE ROBOT CAN SHOOT.
			if(Joperator->GetRawAxis(3)>0)
			{
				rollerPiston->Set(DoubleSolenoid::kReverse);
				isActivated = true;
			}
			else if(Joperator->GetRawAxis(3) == ZERO && isActivated == true)
			{
				rollerPiston->Set(DoubleSolenoid::kForward);
				isActivated = false;
			}
			
			//RIGHT TRIGGER FIRES
			if (Joperator->GetRawAxis(3) < -0.5) {
				rollerPiston->Set(DoubleSolenoid::kReverse);
				Wait(0.5);
				shooter->Set(DoubleSolenoid::kReverse);
				isFired = true;
			} else if(isFired){
				rollerPiston->Set(DoubleSolenoid::kForward);
				Wait(0.25);
				shooter->Set(DoubleSolenoid::kForward);
				isFired = false;
			}
			//D PAD LEFT-RIGHT AXIS CONTROLS ROLLER SPINNING INWARD OR OUTWARD
			if(Joperator->GetRawAxis(6) > 0.5){
				roller->Set(1.0);
				isRolling = true;
			} else if(Joperator->GetRawAxis(6) < -0.5){
				isRolling = true;
				roller->Set(-1.0);
			} else if(isRolling){
				roller->Set(STOP_VALUE);
				isRolling = false;
			}
			//======================Other===================//
			
			 //MAY NOT WORK
			 //IF ARM IS WITHIN ROBOT, STOP.
			/*if (findAngle(angle->GetVoltage())>= 140) 
			{
				moveArmToAngle(135, arm, angle, !inAuto, inTeleop);
				arm->Set(STOP_VALUE);
			}*/
			
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() 
	{

	}

};

START_ROBOT_CLASS(RobotDemo);
