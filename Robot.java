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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Robot extends TimedRobot {

  // Vision Aiming
  TapeThread tapeThread;
  int previousX;
  boolean hasFrame;
  double visionDegrees = 0;
  AHRS ahrs;

  // Controllers
  Joystick leftJoyStick, rightJoyStick;
  XboxController operatorController;

  // Camera
  CameraServer cameraServer;
  UsbCamera camera;

  // Motors
  // Drive
  WPI_TalonSRX frontLeft, rearLeft, frontRight, rearRight;
  DifferentialDrive mainDrive;
  double leftPower, rightPower;
  int driveMode = 0; // For choosing drive mode: 0 is open field (regular), 1 is auton, 2 is scoring

  // Elevator elevatorWinch
  TalonSRX elevatorWinch;

  // Shooter
  WPI_TalonSRX shooter;
  DigitalInput shooterLimit;
  boolean hasCargo; // To tell the operator we have a ball: based on the limit switch

  // Intake
  WPI_TalonSRX intakeGearbox;
  PWMVictorSPX intakeWheels;

  // HAB Lift
  Spark elevatorWinchHAB;
  PWMVictorSPX wheelsHAB;

  // HP Mech
  DoubleSolenoid mechUpDown, hatchGrabRelease;
  boolean hasHatch = false; // To tell the operator we have a hatch
  boolean hatchMechUp = true; // To print the hatch mech's position to SB

  @Override
  public void robotInit() {

    // Vision Aiming
    tapeThread = new TapeThread();
    tapeThread.start();
    ahrs = new AHRS(SPI.Port.kMXP);

    // Controllers
    leftJoyStick = new Joystick(0);
    rightJoyStick = new Joystick(1);
    operatorController = new XboxController(2);

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

    // Elevator winch
    elevatorWinch = new WPI_TalonSRX(5);
    winchInit();

    // Shooter
    shooter = new WPI_TalonSRX(6);
    shooterLimit = new DigitalInput(2);
    hasCargo = !shooterLimit.get();

    // Intake
    intakeGearbox = new WPI_TalonSRX(7);
    intakeWheels = new PWMVictorSPX(0);
    intakeInit();

    // HAB Lift
    elevatorWinchHAB = new Spark(2);
    wheelsHAB = new PWMVictorSPX(1);

    // HP Mech
    mechUpDown = new DoubleSolenoid(0, 1);
    hatchGrabRelease = new DoubleSolenoid(2, 3);
    


  }
  int counter = 0;
  @Override
  public void robotPeriodic() {
    counter = (counter + 1) % 10; 
    if(counter == 0) {
    SmartDashboard.putBoolean("Has Hatch Panel", hasHatch);
    SmartDashboard.putBoolean("Has Cargo", hasCargo);
    if(hatchMechUp) {
      SmartDashboard.putString("Hatch Mech Position", "Up");
    } else {
      SmartDashboard.putString("Hatch Mech Position", "Down");
    }
    SmartDashboard.putString("Target Position", ElevatorConstants.SETPOINT_NAMES[elevatorIndexValue]);
    SmartDashboard.putString("Target Position", IntakeConstants.SETPOINT_NAMES[intakeIndexValue]);
  }
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
    leftPower = deadzoneComp(leftJoyStick.getY()) * -1;
    rightPower = deadzoneComp(rightJoyStick.getY()) * -1; 

    if(rightJoyStick.getRawButton(11)) {
      visionLogic();
      SmartDashboard.putString("Auton Drive", "Robot is in Control");
    } else if(leftJoyStick.getRawButton(6)) {
      scoringDrive();
      SmartDashboard.putString("Scoring Drive", "Manual Scoring");
    } else {
      mainDrive.tankDrive(leftPower, rightPower);
     SmartDashboard.putString("Open Field Drive", "Regular");
    }
  }

  public void auxilaries(){ // Method containing all mechs- eventually elevator

    double shooterCommand = deadzoneComp(operatorController.getRawAxis(1)); // Left Stick on XBOX

    // Shooter Control (left Xbox joystick)
    if(shooterCommand > 0 && shooterLimit.get()) { // Shooter will intake as long as limit isn't pressed
      // intake
      shooter.set(shooterCommand);
    } else {
      shooter.set(shooterCommand);
    }
  
    // HP Grab and Release (left and right bumpers)
    if(operatorController.getRawButton(5)) { // Left Bumper
      // drop it like it's hot...
      hatchGrabRelease.set(DoubleSolenoid.Value.kReverse);
      hasHatch = false;
    } else if(operatorController.getRawButton(6)) { // Right bumper
      // grab
      hatchGrabRelease.set(DoubleSolenoid.Value.kForward);
      hasHatch = true;
    }
     
    // HP Mech Up and Down
    if(operatorController.getRawButton(7)) { // Back button
      // Up
      mechUpDown.set(DoubleSolenoid.Value.kForward);
      hatchMechUp = true;
    } else if(operatorController.getRawButton(8)) { // Start button
      // Down
      mechUpDown.set(DoubleSolenoid.Value.kReverse);
      hatchMechUp = false;
    }

    // HAB elevatorWinch
    double HABelevatorWinchControl = deadzoneComp(operatorController.getRawAxis(5) * -1);
    if(operatorController.getRawButton(10)) { // Have to press down stick AND push it up or down
      elevatorWinchHAB.set(HABelevatorWinchControl); // Moves HAB elevatorWinch up and down
    } else {
      elevatorWinchHAB.set(0);
    }

    double HABdriveControl = deadzoneComp(operatorController.getTriggerAxis(GenericHID.Hand.kLeft));
    // HAB Drive
    wheelsHAB.set(HABdriveControl);
  }

  // Intake wheels
  double intakeWheelsControl = deadzoneComp(operatorController.getTriggerAxis(GenericHID.Hand.kRight));

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
  double d_power = scoringDriveCalc(leftJoyStick.getY() * -1);
  double d_rotation = scoringDriveCalc(rightJoyStick.getX() * -1);
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
  
  // Just a simple dead zone
  public double deadzoneComp(double x){
    if(Math.abs(x) < 0.08){
      return 0;
    }else{
      return x;
    }
  }

  public boolean invertBoolean(boolean input) { // Returns true when input is false and vice versa
    if(input) {
      return false;
    } else if(!input) {
      return true;
    } else {
      return false;
    }
  }

    double elevatorHeading = 0;//Lift target
    int elevatorIndexValue = 0;//Setpoints array address value
  public void elevatorPID() {
    
    //Elevator PID Control
    
    //Go to absolute lift bottom
    if(operatorController.getPOV() == 90) {
      elevatorHeading = ElevatorConstants.SETPOINTS[0];
    }

    //Go to rocket level 3
    if(operatorController.getPOV() == 270) {
      elevatorHeading = ElevatorConstants.SETPOINTS[ElevatorConstants.SETPOINTS.length - 1];
    }

    //Lift moves up one setpoint
    if(operatorController.getPOV() == 0 && elevatorIndexValue <= (ElevatorConstants.SETPOINTS.length - 1)) {
      elevatorHeading = ElevatorConstants.SETPOINTS[++elevatorIndexValue];
    }

    //Lift moves down one setpoint
    if(operatorController.getPOV() == 180 && elevatorIndexValue >= 0) {
      elevatorHeading = ElevatorConstants.SETPOINTS[--elevatorIndexValue];
    }
    
    //Move elevator into the desired position
    elevatorWinch.set(ControlMode.Position,toClicks(elevatorHeading));
    

    //For initial testing
    elevatorWinch.set(ControlMode.PercentOutput, operatorController.getRawAxis(1));//figure out motor direction
    elevatorWinch.getSensorCollection().getPulseWidthPosition();//figure out sensor direction

  }

  public static int toClicks(double inches) {
    return (int) (inches / ElevatorConstants.DRUM_CIRCUMFERENCE * ElevatorConstants.CPR);
  }

  public void winchInit() { // Set up elevator winch PID
     //set elevatorWinch to do nothing at the beginning
     elevatorWinch.set(ControlMode.PercentOutput,0);

     //whether to invert motor direction
     elevatorWinch.setInverted(ElevatorConstants.MOTOR_INVERT);

     //start in brake mode, for regenerative braking.
     elevatorWinch.setNeutralMode(NeutralMode.Brake);    

     //fastest time period where speed can be ramped from 0 to 1. In seconds.
     //elevatorWinch.configClosedloopRamp(.1);     

     //set the maximum speed for the controller (percent of max speed)
     //elevatorWinch.configPeakOutputForward(.5);
     //elevatorWinch.configPeakOutputReverse(.5);

     //can set the controller to adjust power based on battery voltage automatically.
     //elevatorWinch.enableVoltageCompensation();

     //Sensor config
     elevatorWinch.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, ElevatorConstants.TIMEOUT);
     elevatorWinch.setSensorPhase(ElevatorConstants.SENSOR_INVERT);
     
     //nominal/peak values
     elevatorWinch.configNominalOutputForward(0);
     elevatorWinch.configNominalOutputReverse(0);
     elevatorWinch.configPeakOutputForward(1);
     elevatorWinch.configPeakOutputReverse(-1);

     //Configure gain values
     elevatorWinch.config_kF(ElevatorConstants.PID_SLOT, ElevatorConstants.kF);
     elevatorWinch.config_kP(ElevatorConstants.PID_SLOT, ElevatorConstants.kP);
     elevatorWinch.config_kI(ElevatorConstants.PID_SLOT, ElevatorConstants.kI);
     elevatorWinch.config_kD(ElevatorConstants.PID_SLOT, ElevatorConstants.kD);


     int absolutePosition = elevatorWinch.getSensorCollection().getPulseWidthPosition();

     /* Mask out overflows, keep bottom 12 bits */
     absolutePosition &= 0xFFF;
     if (ElevatorConstants.SENSOR_INVERT) { absolutePosition *= -1; }
     if (ElevatorConstants.MOTOR_INVERT) { absolutePosition *= -1; }
     
     /* Set the quadrature (relative) sensor to match absolute */
     elevatorWinch.setSelectedSensorPosition(absolutePosition, 0, ElevatorConstants.TIMEOUT);
  }

  double intakeHeading = 0;//Intake target
  int intakeIndexValue = 0;//Setpoints array address value

  public void intakeInit() {
     //set intakeGearbox to do nothing at the beginning
     intakeGearbox.set(ControlMode.PercentOutput,0);
     //whether to invert motor direction
     intakeGearbox.setInverted(IntakeConstants.MOTOR_INVERT);
     //start in brake mode, for regenerative braking.
     intakeGearbox.setNeutralMode(NeutralMode.Brake);
     
     //fastest time period where speed can be ramped from 0 to 1. In seconds.
     //intakeGearbox.configClosedloopRamp(.1);
     
     //set the maximum speed for the controller (percent of max speed)
     //intakeGearbox.configPeakOutputForward(.5);
     //intakeGearbox.configPeakOutputReverse(.5);

     //can set the controller to adjust power based on battery voltage automatically.
     //intakeGearbox.enableVoltageCompensation();

     //Sensor config
     intakeGearbox.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, IntakeConstants.TIMEOUT);
     intakeGearbox.setSensorPhase(IntakeConstants.SENSOR_INVERT);
     
     //nominal/peak values
     intakeGearbox.configNominalOutputForward(0);
     intakeGearbox.configNominalOutputReverse(0);
     intakeGearbox.configPeakOutputForward(1);
     intakeGearbox.configPeakOutputReverse(-1);

     //Configure gain values
     intakeGearbox.config_kF(IntakeConstants.PID_SLOT, IntakeConstants.kF);
     intakeGearbox.config_kP(IntakeConstants.PID_SLOT, IntakeConstants.kP);
     intakeGearbox.config_kI(IntakeConstants.PID_SLOT, IntakeConstants.kI);
     intakeGearbox.config_kD(IntakeConstants.PID_SLOT, IntakeConstants.kD);


     int absolutePosition = intakeGearbox.getSensorCollection().getPulseWidthPosition();

     /* Mask out overflows, keep bottom 12 bits */
     absolutePosition &= 0xFFF;
     if (IntakeConstants.SENSOR_INVERT) { absolutePosition *= -1; }
     if (IntakeConstants.MOTOR_INVERT) { absolutePosition *= -1; }
     
     /* Set the quadrature (relative) sensor to match absolute */
     intakeGearbox.setSelectedSensorPosition(absolutePosition, 0, IntakeConstants.TIMEOUT);
     
     //TODO: Force into starting position, then set baseline relative position? See the sample code:
     //https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/PositionClosedLoop/src/main/java/frc/robot/Robot.java
 }
  

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
