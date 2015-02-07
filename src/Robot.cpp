#include "WPILib.h"

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public SampleRobot
{

    // Channels for the wheels
    const static int frontLeftChannel	= 2;
    const static int rearLeftChannel	= 4;
    const static int frontRightChannel	= 6;
    const static int rearRightChannel	= 0;

    const static int elevatorChannel = 1;
    const static int elevatorPotChannel = 3;

    const static int joystickChannel	= 0;

    const static int encoderChannelA1 = 0;
    const static int encoderChannelB1 = 1;
    const static int encoderChannelA2 = 2;
    const static int encoderChannelB2 = 3;

    const static int limitSwitchChannel = 9;

    float finalAngle;

	RobotDrive robotDrive;			// robot drive system
	Joystick stick;					// only joystick
	Gyro gyro;						//gyro 1
	BuiltInAccelerometer accel;   	//built in accelerometer
	AnalogPotentiometer elevatorPot; 		//potentiometer
	DigitalInput limitSwitch; 		//limit switch
	Talon elevator;
	Encoder encoder1;
	Encoder encoder2;
	PIDController pid;
	//PowerDistributionPanel power;


public:
	Robot() :
			robotDrive(frontLeftChannel, rearLeftChannel,
					   frontRightChannel, rearRightChannel),	// these must be initialized in the same order
			stick(joystickChannel),								// as they are declared above.
			gyro(0),
			accel(),
			elevatorPot(elevatorPotChannel, 10, 0),
			limitSwitch(limitSwitchChannel),
			elevator(elevatorChannel),
			encoder1(encoderChannelA1, encoderChannelB1),
			encoder2(encoderChannelA2, encoderChannelB2),
			pid(0.05, 0, 0, (PIDSource *)&gyro, (PIDOutput *)&finalAngle)
	{
		robotDrive.SetExpiration(0.1);
		robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);	// invert the left side motors
		robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);	// you may need to change or remove this to match your robot
	}

	/*
	 * Runs the motors with Mecanum drive.
	 */
	void OperatorControl()
	{
		const static float elevatorGap = 0.1;

		float angle, xAxis, yAxis, zAxis;
		float joy_atten;
		double x, y, z, rotation;
		bool lSwitch;
		bool antinitrus;
		bool gyroReset;
		float testStop = 3.4;
		float minStop, maxStop;

		double encoderData1, encoderData2;

		//gyro.InitGyro();
		gyro.Reset();
		gyro.SetSensitivity(0.0078);
		gyro.SetDeadband(0.005);

		robotDrive.SetSafetyEnabled(false);

		elevator.Set(0.5);
		minStop = testStop - elevatorGap;
		maxStop = testStop + elevatorGap;

		encoder1.Reset();
		encoder2.Reset();
		//encoder1.SetDistancePerPulse(1);


		while (IsOperatorControl() && IsEnabled())
		{
			angle = gyro.GetAngle();
			x = accel.GetX();
			y = accel.GetY();
			z = accel.GetZ();

			encoderData1 = encoder1.GetRate();
			encoderData2 = encoder2.GetRate();


			//Gyro Reset Button
			gyroReset = stick.GetRawButton(7);

			if (gyroReset == true)
			{
				gyro.Reset();
			}

			//Speed Reduction Button
			antinitrus = stick.GetRawButton(8);

			if (antinitrus == true)
			{
				//Sets speed to 0.5 power while holding button
				joy_atten = 0.5;
			}
			else
			{
				joy_atten = 1;
			}

			xAxis = stick.GetX() * joy_atten;
			yAxis = stick.GetY() * joy_atten;
			zAxis = stick.GetZ() * joy_atten;

        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
			robotDrive.MecanumDrive_Cartesian(xAxis, yAxis, finalAngle, angle);

			//Elevator Potentiometer Speed
			rotation = elevatorPot.Get();

			bool elevatorMotor;
			elevatorMotor = stick.GetRawButton(1);

			if (elevatorMotor == true)
			{
				elevator.Set(0.25);
			}

			/*if (rotation < minStop)
			{
				elevator.Set(0.1);
			}
			//else if (rotation > maxStop)
			{
				elevator.Set(-0.1);
			}
			//else
			{
				elevator.Set(0);
			}
			 */
			//lSwitch = limitSwitch.Get();

			//double rearRight;
			//rearRight = power.GetCurrent(0);

			//PID Crap
			float setpointAngle;

			pid.Enable();


			if(zAxis != 0)
			{
				pid.Disable();
				setpointAngle = 6669;
				finalAngle = zAxis;
			}


			if(zAxis == 0 && setpointAngle == 6669)
			{
				setpointAngle = gyro.GetAngle();
			}

			pid.SetSetpoint(setpointAngle);
			//pid.PIDController(.05, 0, 0, angle, finalAngle);


			//Smart Dashboard Stuff
			SmartDashboard::PutNumber("Gyro ", finalAngle);
			SmartDashboard::PutNumber("Accelerometer X ", x);
			SmartDashboard::PutNumber("Accelerometer Y ", y);
			SmartDashboard::PutNumber("Accelerometer Z ", z);
			SmartDashboard::PutNumber("Potentiometer ", rotation);
			SmartDashboard::PutBoolean("Limit Switch ", lSwitch);
			SmartDashboard::PutNumber("X axis ", xAxis);
			SmartDashboard::PutNumber("Y axis ", yAxis);
			SmartDashboard::PutNumber("Z axis ", zAxis);
			SmartDashboard::PutNumber("Encoder 1 ", encoderData1);
			SmartDashboard::PutNumber("Encoder 2 ", encoderData2);
			//SmartDashboard::PutNumber("Rear Right Drive ", rearRight);

			Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}

};

START_ROBOT_CLASS(Robot);
