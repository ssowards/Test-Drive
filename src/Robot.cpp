#include "WPILib.h"

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public SampleRobot
{

    // Channels for the wheels
    const static int frontLeftChannel	= 2;
    const static int rearLeftChannel	= 3;
    const static int frontRightChannel	= 1;
    const static int rearRightChannel	= 0;

    const static int joystickChannel	= 0;

	RobotDrive robotDrive;	// robot drive system
	Joystick stick;			// only joystick
	Gyro gyro;				//gyro
	BuiltInAccelerometer accel;   //built in accelerometer
	AnalogPotentiometer pot; //potentiometer
	DigitalInput limitSwitch; //limit switch

public:
	Robot() :
			robotDrive(frontLeftChannel, rearLeftChannel,
					   frontRightChannel, rearRightChannel),	// these must be initialized in the same order
			stick(joystickChannel),								// as they are declared above.
			gyro(0),
			accel(),
			pot(3, 10, 0),
			limitSwitch(0)
	{
		robotDrive.SetExpiration(0.1);
		robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);	// invert the left side motors
		robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);	// you may need to change or remove this to match your robot
	}

	/**
	 * Runs the motors with Mecanum drive.
	 */
	void OperatorControl()
	{
		float angle;
		double x, y, z, rotation;
		bool lSwitch;

		gyro.Reset();
		gyro.SetDeadband(0.005);
		robotDrive.SetSafetyEnabled(false);

		while (IsOperatorControl() && IsEnabled())
		{
			angle = gyro.GetAngle();
			x = accel.GetX();
			y = accel.GetY();
			z = accel.GetZ();
			rotation = pot.Get();
			lSwitch = limitSwitch.Get();

        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
			robotDrive.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(), stick.GetZ(), angle);
			SmartDashboard::PutNumber("Gyro ", angle);
			SmartDashboard::PutNumber("Accelerometer X ", x);
			SmartDashboard::PutNumber("Accelerometer Y ", y);
			SmartDashboard::PutNumber("Accelerometer Z ", z);
			SmartDashboard::PutNumber("Potentiometer ", rotation);
			SmartDashboard::PutBoolean("Limit Switch ", lSwitch);
			Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}

};

START_ROBOT_CLASS(Robot);
