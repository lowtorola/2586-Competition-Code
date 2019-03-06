/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {

  // Vision Aiming
  TapeThread tapeThread;
  int previousX;
  boolean hasFrame;
  double visionDegrees = 0;
  AHRS ahrs;

  // Controllers
  Joystick leftStick, rightStick;
  XboxController buttonBoard;

  // Camera
  CameraServer cameraServer;
  UsbCamera camera;

  // Motors
  // Drive
  WPI_TalonSRX frontLeft, rearLeft, frontRight, rearRight;
  DifferentialDrive mainDrive;
  double leftPower, rightPower;
  int scoringMode = 0; // For choosing drive mode: 0 is open field (regular), 1 is auton, 2 is scoring

  // Elevator Winch
  WPI_TalonSRX elevatorWinch;

  // Shooter
  WPI_TalonSRX shooter;
  DigitalInput shooterLimit;

  // Intake
  WPI_TalonSRX intakeGearbox;
  PWMVictorSPX intakeWheels;

  // HAB Lift
  Spark winchHAB;
  PWMVictorSPX wheelsHAB;

  // HP Mech
  DoubleSolenoid mechUpDown, hatchGrabRelease;

  @Override
  public void robotInit() {

    // Vision Aiming
    tapeThread = new TapeThread();
    tapeThread.start();
    ahrs = new AHRS(SPI.Port.kMXP);

    // Controllers
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
    buttonBoard = new XboxController(2);

    // Motors
    // Drive
    frontLeft = new WPI_TalonSRX(1);
    rearLeft = new WPI_TalonSRX(2);
    frontRight = new WPI_TalonSRX(4);
    rearRight = new WPI_TalonSRX(3);
    rearLeft.set(ControlMode.Follower, frontLeft.getBaseID());
    rearRight.set(ControlMode.Follower, frontRight.getBaseID());

    mainDrive = new DifferentialDrive(frontLeft, frontRight);

    // Camera
    camera = CameraServer.getInstance().startAutomaticCapture();
   camera.setResolution(300, 225);
   camera.setFPS(30);

    // Elevator Winch
    elevatorWinch = new WPI_TalonSRX(5);

    // Shooter
    shooter = new WPI_TalonSRX(6);
    shooterLimit = new DigitalInput(2);

    // Intake
    intakeGearbox = new WPI_TalonSRX(7);
    intakeWheels = new PWMVictorSPX(0);

    // HAB Lift
    winchHAB = new Spark(2);
    wheelsHAB = new PWMVictorSPX(1);

    // HP Mech
    mechUpDown = new DoubleSolenoid(0, 1);
    hatchGrabRelease = new DoubleSolenoid(2, 3);
    


  }

  @Override
  public void robotPeriodic() {
  }


  @Override
  public void autonomousInit() {
   
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  teleopPeriodic();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    auxilaries();
    leftPower = deadzoneComp(leftStick.getY()) * -1;
    rightPower = deadzoneComp(rightStick.getY()) * -1; 

    if(rightStick.getTrigger()) {
      scoringMode = 1;
    } else if(leftStick.getTrigger()) {
      scoringMode = 2;
    } else {
      scoringMode = 0;
    }

   switch(scoringMode) {
     case 0:
     mainDrive.tankDrive(leftPower, rightPower);
     SmartDashboard.putString("Open Field Drive", "Regular"); // TODO: Is this how I would print out on SmartDash what drive mode I am in?
     break;

     case 1:
     visionLogic();
     SmartDashboard.putString("Auton Drive", "Robot is in Control");
     break;

     case 2:
     scoringDrive();
     SmartDashboard.putString("Scoring Drive", "Manual Scoring");
     break;
   }
   scoringMode = 0; // TODO: Make sure this is needed. This resets to normal drive
  }

  public void auxilaries(){ // Method containing all mechs- eventually elevator

    // Shooter Control
    if(buttonBoard.getRawAxis(1) > 0 && shooterLimit.get()) { // Shooter will intake as long as limit isn't pressed
      // intake
      shooter.set(-1);
    } else if(buttonBoard.getRawAxis(1) < 0) {
      // outtake
      shooter.set(1);
    }else{
      shooter.set(0);
    }

    // HP Grab and Release
    if(buttonBoard.getRawAxis(0) > 0) {
      // release
      hatchGrabRelease.set(DoubleSolenoid.Value.kReverse);
    } else if(buttonBoard.getRawAxis(0) < 0) {
      // grab
      hatchGrabRelease.set(DoubleSolenoid.Value.kForward);
    }
     
    // HP Mech Up and Down
    if(buttonBoard.getRawButton(9)) {
      // Up
      mechUpDown.set(DoubleSolenoid.Value.kForward);
    } else if(buttonBoard.getRawButton(10)) {
      // Down
      mechUpDown.set(DoubleSolenoid.Value.kReverse);
    }

    // HAB Winch
    if(leftStick.getRawButton(6) && rightStick.getRawButton(6)) { // Multiple buttons to prevent accidental activation
      winchHAB.set(-1);
    } else if(leftStick.getRawButton(11) && rightStick.getRawButton(11)) { // Same thing here
      winchHAB.set(1);
    } else {
      winchHAB.set(0);
    }

    // HAB Drive
    if(leftStick.getRawButton(8) && rightStick.getRawButton(8)) { // Multiple buttons to prevent accidental activation
      wheelsHAB.set(1);
    } else if(leftStick.getRawButton(9) && rightStick.getRawButton(9)) {
      wheelsHAB.set(-1);
    } else {
      wheelsHAB.set(0);
    }
}

  // Vision Aiming Code. I didn't program this, so I did my best to implement it. TODO: Debug as necessary
public void visionLogic(){
/*
  Vision logic for determining what to do while vision mode is enabled
  1. Compare to see if the center of the rectangle is outside of ideal conditions
  2. If a frame isn't grabbed, grab one and PID to it
  3. Once aligned, move forward at adjusted speed based on right joystick 

  54* across 800 pixels = .0675 degrees/pixel

*/
  if(tapeThread.x != previousX){
    hasFrame = false;
  }
     previousX = tapeThread.x;
    if(Math.abs(tapeThread.x) > 40 && !hasFrame){
      grabFrame();
    }else if(hasFrame){
      mainDrive.arcadeDrive(0, pidTurn(visionDegrees));
    }else if(Math.abs(tapeThread.x) < 40){
      mainDrive.arcadeDrive(rightPower, 0.0);
    }

}

public static double checkAgainstFloor(double x){
  //Used for ensuring that motor outputs are greater than the force of friction holding the robot still

  double floor = .5;
  if(Math.abs(x) < floor){
    if(x>0){return floor;}else{return -floor;}
  }else{
    return x;
  }
}

public void grabFrame(){
  int rawX = tapeThread.x;
  visionDegrees = rawX * .0675;
  visionDegrees = ahrs.getAngle() + visionDegrees;
  hasFrame = true;

}

public double pidTurn(double target){
    double kP_rot = 1/90;
    return target * kP_rot;
}

public static CameraServer getCameraServer(){
  return CameraServer.getInstance();
}

public void compareValsWithin(double zone){
  if(Math.abs(leftPower - rightPower) < zone){
      double newValue = (leftPower + rightPower)/2;
      leftPower = newValue;
      rightPower = newValue;
  }
}

// Smooth "Scoring Drive" for alignment
public void scoringDrive() {
  double d_power = scoringDriveCalc(leftStick.getY() * -1);
  double d_rotation = scoringDriveCalc(rightStick.getX() * -1);
  mainDrive.arcadeDrive(d_power, d_rotation);
}

// Calculation for smooth "Scoring Drive"
  public double scoringDriveCalc(double input) {
    if(Math.abs(input) < 0.08){
      return 0;
    }else{
      return input * 0.85;
    }
  }
  
  // Calculation for "Open Field Drive." Just a simple dead zone
  public double deadzoneComp(double x){
    if(Math.abs(x) < 0.08){
      return 0;
    }else{
      return x;
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
