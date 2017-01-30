// This is the code for 2016 FRC Robot

package org.usfirst.frc.team5493.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import static java.lang.Math.*;



public class Robot extends SampleRobot {
	CameraServer server;
	 public Robot() {
		 server = CameraServer.getInstance();
		 server.setQuality(50);
		 //the camera name (ex "cam0") can be found through the roborio web interface
		 server.startAutomaticCapture("cam0");

		 // Create a robot drive object using CAN ID's
		 CANTalon driveRightFront = new CANTalon(1); // CAN port 1 - Right Front
		 CANTalon driveRightRear = new CANTalon(5); // CAN port 3 - Right Back
		 CANTalon driveLeftRear = new CANTalon(11); // CAN port 4 - Left Rear
		 CANTalon driveLeftFront = new CANTalon(9); // CAN port 10 - Left Front

		 RobotDrive tank_drive = new RobotDrive(driveLeftFront, driveLeftRear, driveRightFront, driveRightRear);

		 Talon shootLeft = new Talon(9); // PWM port 9
		 Talon shootRight = new Talon(8); // PWM port 8

		 CANTalon angleShoot = new CANTalon(7);
		 CANTalon liftMotor = new CANTalon(3);

		 // Define joysticks being used at USB port 0 and 2 on the Driver Station
		 Joystick logiTech = new Joystick(0);
		 Joystick logiTech2 = new Joystick(2);
		 // AnalogInput rSensor = new AnalogInput(1);
		 // AnalogInput lSensor = new AnalogInput(2);
		 AnalogInput dSensor = new AnalogInput(3);

		 DigitalInput limitForShooter = new DigitalInput(9);

		 // Define compressor and doublesolenoid
		 Compressor compressor = new Compressor(0);
		 DoubleSolenoid doublesolenoid = new DoubleSolenoid(1, 0);
		 private DriverStation station = DriverStation.getInstance();

		 // buttons on the joystick
		 // axis buttons control the cylinder
		 // raw buttons control the intake/shoot wheels
		 // right side of joystick for shooting
		 private final int shoot = 6;
		 private final int cylinderOut = 3;
		 // left side of joystick for intake
		 private final int intake = 5;
		 private final int cylinderIn = 2;
		 public boolean shooterOn = false;
		 public boolean intakeOn = false;
		 public boolean angleUpOn = false;
		 public boolean angleDownOn = false;
		 //	public boolean isPressedShoot = false;
		 //	public boolean isPressedIntake = false;
		 //	boolean didSolenoid = false;

		 private final int resetAngle = 4; //yellow
		 private final int distanceCalc = 3; //blue
		 private final int doStuff = 2; //red

		 long intakeCount = 0;
		 long shootCount = 0;
		 long timer = 0;
		 // Gear motor am-2971
		 Encoder enc = new Encoder(7, 8, false, Encoder.EncodingType.k4X);

		 boolean cameraOn = false;
	 }
	 
	 protected void robotInit() {
		 this.tank_drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
		 this.tank_drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
	 }

	public void autonomous() {
		while (true && isOperatorControl() && isEnabled()) {
			//autoShoot();
			//launcher();
			// encoder();
			break;
		}
	 }
	//	private void autoShoot(){
	//		//drive forward for two seconds
	//		driveRightFront.set(0.5);
	//		driveRightRear.set(0.5);
	//		driveLeftFront.set(0.5);
	//		driveLeftRear.set(0.5);
	//		Timer.delay(2.0);
	//		//turn the robot to some angle so it is aligned and ready to shoot
	//		driveRightFront.set(0.1);
	//		driveRightRear.set(0.1);
	//		driveLeftFront.set(0.5);
	//		driveLeftRear.set(0.5);
	//		Timer.delay(2.0);
	//		driveRightFront.set(0.0);
	//		driveRightRear.set(0.0);
	//		driveLeftFront.set(0.0);
	//		driveLeftRear.set(0.0);
	//		//angle up for half a second
	//		angleShoot.set(0.5);
	//		Timer.delay(0.5);
	//		//start shoot wheels
	//		shootLeft.set(0.75);
	//		shootRight.set(-0.75);
	//		//cylinderOut
	//		doublesolenoid.set(DoubleSolenoid.Value.kForward);
	//		//turn off wheels
	//		shootLeft.set(0.0);
	//		shootRight.set(0.0);
	//	}

	private void encoder() {
		/*
		 * Count - Current count. May be reset by calling reset() 
		 * Raw Count - Count without compensation for decoding scale factor. 
		 * Distance - Current distance reading from counter.(count * Distance Per Count scale factor)
		 * Rate - Current rate of the counter in units/sec. It is calculated using the
		 * DistancePerPulse divided by the period. If the counter is stopped
		 * this value may return Inf or NaN, depending on language. 
		 * Direction - The direction of the last value change (true for Up, false for Down)
		 * Stopped - If the counter is currently stopped (period has exceeded Max Period)
		 */

		//		 int count = enc.get(); double distance = enc.getRaw(); double
		//		 distance1 = enc.getDistance(); double rate = enc.getRate(); boolean
		//		 direction = enc.getDirection(); boolean stopped = enc.getStopped();

	}

	public void encoder1() {
		double count = enc.get();
		double angle = Math.pow(-0.00120526087267*count, 2) + (.776298*count) - (48.456164);
		DriverStation.reportError(count + " " + angle, false);
	}

	private void launcher() {
		// channel A = port # 8
		// channel B = port # 7
		// for (int i = 0; i < 1; i++){
		// launch boulder with specific position, speed of wheels, and angle
		// need to find this when programming parts of the robot
		// }
		while (dSensor.getValue() > 50) {
			double sensor = dSensor.getValue();
			double sensor2 = (sensor / 400);
			double test = (2.1082 / sensor2);
			Integer test2 = (int) Math.round(test);
			double simple = atan(test2);
			double simple2 = Math.toDegrees(simple);
			Integer simple3 = (int) Math.round(simple2);
			double theta = (15 + simple3);
			Integer theta2 = (int) Math.round(theta);
			String theta3 = theta2.toString();
			DriverStation.reportError(theta3, false);
			Timer.delay(1);
		}
	}
	// Gabrielle, I used this to get an angle measure for which we should shoot
	// at, it's moderately accurate. But I can't get it to show me anything when
	// in autonomous.

	public void operatorControl() {
		while (true && isOperatorControl() && isEnabled()) {
//			if (isOperatorControl()){
//
//			}


			// initialize tankDrive (inversed)
			tank_drive.tankDrive(logiTech.getRawAxis(DRIVELEFT) * -1, logiTech.getRawAxis(DRIVERIGHT) * -1);

			// intake wheels
			if (logiTech2.getRawButton(intake) && !intakeOn) {
				doublesolenoid.set(DoubleSolenoid.Value.kForward);
				intakeOn = true;
				Timer.delay(0.20);
				DriverStation.reportError("INTAKE ON ON ON Intake ", false);
				intakeCount = System.currentTimeMillis();
				shootLeft.set(-0.50);
				shootRight.set(0.50);
				//				isPressedIntake = false;
			} else if (intakeOn && (logiTech2.getRawButton(intake))) {
				intakeOn = false;
				shootLeft.set(0.0);
				shootRight.set(0.0);
				DriverStation.reportError("INTAKE OFF OFF OFF Intake ", false);
				Timer.delay(0.20);
				// DriverStation.reportError(" Intake OFF ", false);
				//				isPressedIntake = false;
			}

			// shoot wheels
			if (logiTech2.getRawButton(shoot) && !shooterOn) {
				shooterOn = true;
				doublesolenoid.set(DoubleSolenoid.Value.kForward);
				Timer.delay(0.20);
				DriverStation.reportError("SHOOTER ON ON ON Shoot ", false);
				shootCount = System.currentTimeMillis();
				shootLeft.set(0.75);
				shootRight.set(-0.75);
				//				isPressedShoot = false;
			} else if ((System.currentTimeMillis() - shootCount > 2000)
					&& (System.currentTimeMillis() - shootCount < 2999)) {
				//				if (!didSolenoid) {
				doublesolenoid.set(DoubleSolenoid.Value.kReverse);
				DriverStation.reportError("SOLENOID REVERSE REVERSE Shoot ", false);
				//				didSolenoid= true;
				//				}
			} else if (shooterOn && (/*(isPressedShoot && System.currentTimeMillis() - shootCount > 100 )||*/ System.currentTimeMillis() - shootCount > 3000)) {
				shooterOn = false;
				shootLeft.set(0.0);
				shootRight.set(0.0);
				doublesolenoid.set(DoubleSolenoid.Value.kForward);
				Timer.delay(0.20);
				DriverStation.reportError("SHOOTER OFF OFF OFF Shoot ", false);
				//				isPressedShoot = false;
				//				didSolenoid = false;
			}

			// cylinder for shooting the boulder
			if (logiTech2.getRawAxis(cylinderIn) > 0) {
				doublesolenoid.set(DoubleSolenoid.Value.kForward);
			} else if (logiTech2.getRawAxis(cylinderOut) > 0) {
				doublesolenoid.set(DoubleSolenoid.Value.kReverse);
			} else {
				doublesolenoid.set(DoubleSolenoid.Value.kOff);
			}

			if (logiTech2.getRawAxis(5) < -0.15) {
				if (!angleUpOn && !angleDownOn) {
					angleUpOn = true;
					angleShoot.set(-0.75);
					// Timer.delay(.01);
				}
			} else if (logiTech2.getRawAxis(5) > 0.25 && limitForShooter.get()) {
				if (!angleDownOn && !angleUpOn) {
					angleDownOn = true;
					angleShoot.set(0.25);
					// Timer.delay(.01);
				}
			} else {
				angleDownOn = false;
				angleUpOn = false;
				angleShoot.set(0.0);
				// Timer.delay(.01);
			}
			// following if loop- make into a method that is called at beginning
			// of autonomous
			if (logiTech2.getRawButton(resetAngle)) {
				//yellow
				if(!limitForShooter.get()){
					enc.reset();
				}
				double count = enc.get();
				double printAngle = Math.pow(0.000014625015 * count, 2) + (0.2629431379*count) - (22.19041769);
				//when count = 0, angle = 0
				double angle = Math.pow(0.000014625015 * count, 2) + (0.2629431379*count);

				DriverStation.reportError(" COUNT: " + count + " ANGLE: " + printAngle, false);
				//DriverStation.reportError(enc.get() + "*", false);
			}

			if (logiTech2.getRawButton(distanceCalc)){
				//blue
				double sensor = dSensor.getValue();
				double sensor2 = (sensor / 400);
				double test = (2.1082 / sensor2);
				Integer test2 = (int) Math.round(test);
				double simple = atan(test2);
				double simple2 = Math.toDegrees(simple);
				Integer simple3 = (int) Math.round(simple2);
				double theta = (15 + simple3);
				Integer theta2 = (int) Math.round(theta);
				String theta3 = theta2.toString();
				double distance = dSensor.getValue();
				double distanceMeters = (distance / 365) + 1;
				DriverStation.reportError(" Distance: " + distanceMeters, false);
			}

			if(logiTech2.getRawButton(doStuff)){
				//red
				//angle is the current angle
				double count = enc.get();
				double angle = Math.pow(0.000014625015 * count, 2) + (0.2629431379*count) - (22.19041769);
				int printAngle = (int)(Math.round(angle));
				//find the distance away from closest object
				double distance = dSensor.getValue();
				double distanceMeters = (distance / 365) + .984;
				double ratio = (2.1082 / distanceMeters);
				double radians = atan(ratio);
				//convert to degrees
				double desiredDegrees = (radians * (180/Math.PI)) + 15;
				int printDegrees = (int) Math.round(desiredDegrees);
				timer = System.currentTimeMillis();
				DriverStation.reportError("DA: " + printDegrees + " Angle: " + printAngle, false);
				while(System.currentTimeMillis() - timer < 3000){
					double c = enc.get();
					double angleBottom = Math.pow(0.000014625015 * c, 2) + (0.2629431379*c) - (22.19041769);
					if(angleBottom < 45 - 1)
						angleShoot.set(-0.25);
					else if(angleBottom > 45 + 1)
						angleShoot.set(0.25);
					else
						angleShoot.set(0.0);

				}
				DriverStation.reportError(" Angle: " + printAngle, false);
				doublesolenoid.set(DoubleSolenoid.Value.kForward);
				Timer.delay(0.20);
				DriverStation.reportError("SHOOTER ON ", false);
				if (!shooterOn){
					shooterOn = true;
					doublesolenoid.set(DoubleSolenoid.Value.kForward);
					Timer.delay(0.20);
					DriverStation.reportError("SHOOTER ON ON ON Shoot ", false);
					shootCount = System.currentTimeMillis();
					shootLeft.set(1.0);
					shootRight.set(-1.0);
				} else if ((System.currentTimeMillis() - shootCount > 2000)
						&& (System.currentTimeMillis() - shootCount < 2999)) {
					doublesolenoid.set(DoubleSolenoid.Value.kReverse);
					DriverStation.reportError("SOLENOID REVERSE REVERSE Shoot ", false);
				} else if (shooterOn && (System.currentTimeMillis() - shootCount > 3000)) {
					shooterOn = false;
					shootLeft.set(0.0);
					shootRight.set(0.0);
					doublesolenoid.set(DoubleSolenoid.Value.kForward);
					Timer.delay(0.20);
					DriverStation.reportError("SHOOTER OFF OFF OFF Shoot ", false);
				}
			}
			if (logiTech.getRawButton(8)){
				if (!cameraOn)
					cameraOn = true;
				else
					cameraOn = false;
			}

			if (cameraOn){
				server = CameraServer.getInstance();
				server.setQuality(50);
				server.startAutomaticCapture("cam1");
			}
		}
	}
}
