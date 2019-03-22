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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

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
  double d_power, d_rotation;

  // Elevator elevatorWinch
  WPI_TalonSRX elevatorWinch;
  DigitalInput mainElevatorLow; // lift limit switches
	boolean liftBottomLimit;

  // Shooter arm
  PWMVictorSPX shooterTop;
  PWMVictorSPX shooterBottom;
  WPI_TalonSRX armRotator;
  double armGBDegrees;
  double armGBControl;
  double armGBClicks;

  // HAB Lift
  Spark HABDrive;
  DigitalInput HABLiftLow;
  DigitalInput HABLiftHigh;

  @Override
  public void robotInit() {

    // Vision Aiming
    tapeThread = new TapeThread();
  //  tapeThread.start();
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
    SpeedControllerGroup left = new SpeedControllerGroup(frontLeft, rearLeft);
    SpeedControllerGroup right = new SpeedControllerGroup(frontRight, rearRight);


    mainDrive = new DifferentialDrive(left, right);

    // Camera      
    camera = CameraServer.getInstance().startAutomaticCapture();
   camera.setResolution(200, 150);
   camera.setFPS(15);

    // Elevator winch
    elevatorWinch = new WPI_TalonSRX(5);
    mainElevatorLow = new DigitalInput(7);

    // Shooter
    shooterTop = new PWMVictorSPX(0);
    shooterBottom = new PWMVictorSPX(1);
    armRotator = new WPI_TalonSRX(7);
  //  intakeInit(); // Method to set up intake gearbox: set it in its own method for organization

    // HAB Lift
    HABDrive = new Spark(2);
    HABLiftLow = new DigitalInput(8);
    HABLiftHigh = new DigitalInput(9);

  }
  int counter = 0; // Integer to make it so that we print out to ShuffleBoard every 10 cycles
  @Override
  public void robotPeriodic() {
    counter = (counter + 1) % 10; // Increase counter by 1, get the remainder to use later
    if(counter == 0) {

      SmartDashboard.putNumber("Arm Degrees", armGBDegrees);
      SmartDashboard.putNumber("Forward/Back", leftPower);
      SmartDashboard.putNumber("Rotation", d_rotation);
      SmartDashboard.putBoolean("Elevator at Bottom", !liftBottomLimit);


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
    teleopPeriodic(); // Utilize teleOp code here
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    liftBottomLimit = mainElevatorLow.get(); // low limit switch on lift value

    auxilaries(); // Method for controlling mechs
    d_power = squareInput(deadZoneComp(leftJoyStick.getY()) * -1); // TODO: check this
    d_rotation = squareInput(deadZoneComp(rightJoyStick.getX()));

    mainDrive.arcadeDrive(d_power, d_rotation);

  }

  public void auxilaries(){ // Method containing all mechs except for elevator and intake

    // Elevator will stop when it hits the bottom limit
    double liftCommand = squareInput(deadZoneComp(operatorController.getRawAxis(1) * -1)); // Left stick Y
    if (liftCommand > 0) {
      elevatorWinch.set(liftCommand);
    } else if (liftBottomLimit && liftCommand < 0) {
      elevatorWinch.set(liftCommand);
    } else {
      elevatorWinch.set(0);
    }

    double cargoIn = capInput(deadZoneComp(operatorController.getRawAxis(2))); // left trigger
    double cargoOut = capInput(deadZoneComp(operatorController.getRawAxis(3))); // right trigger

  if(operatorController.getRawButton(1)) {
    shooterTop.set(0);
    shooterBottom.set(0);
  } else if(cargoIn > cargoOut) {
      shooterTop.set(cargoIn);
      shooterBottom.set(cargoIn);
    } else if(cargoOut > cargoIn) {
      shooterTop.set(cargoOut * -1);
      shooterBottom.set(cargoOut * -1);
    }
  
  
  //Arm: Controlled by PID(ish) method
    armGBControl = deadZoneComp(operatorController.getRawAxis(5) * -1);
    armGBClicks = armRotator.getSelectedSensorPosition();
    armGBDegrees = clicksToDegrees(armGBClicks);

    int upperLimit = 45;
    int lowerLimit = 90;
    // not sure if it's this
   if (armGBDegrees > upperLimit && armGBControl > 0) {
     armRotator.set(0.25 * (armGBControl) * -1);
   } else if (armGBDegrees < lowerLimit && armGBControl < 0) {
     armRotator.set(0.25 * (armGBControl) * -1);
   } else {
     armRotator.set(0);
   } 
   // or this
/*   if (armGBDegrees <= upperLimit && armGBControl < 0) {
    armRotator.set(0);
  } else if (armGBDegrees >= lowerLimit && armGBControl > 0){
    armRotator.set(0);
  } else {
    armRotator.set(0.75 * (armGBControl));
  } */

  // HAB Lift
  boolean HABLiftTop = HABLiftHigh.get(); // The "high limit switch" is to stop it from going too low. Sounds wrong but should be right
  boolean HABLiftBottom = HABLiftLow.get();
  double HABWinchDrive = deadZoneComp(operatorController.getRawAxis(4));

  if (operatorController.getRawButton(6) && HABWinchDrive > 0 && HABLiftBottom) { // to drive the lift down
    HABDrive.set(HABWinchDrive * 0.25);
  } else if (operatorController.getRawButton(6) && HABWinchDrive < 0 && HABLiftTop) { // to drive the lift up
    HABDrive.set(HABWinchDrive * 0.25);
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
     // mainDrive.arcadeDrive(rightPower, 0.0);
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
  double d_rotation = scoringDriveCalc(rightJoyStick.getX());
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

  // Squares input: for ease-of-driving
  public double squareInput(double input) {
    if(input < 0) {
      return (-1 * (Math.pow(input, 2)));
    } else if (input > 0) {
      return (Math.pow(input, 2));
    } else {
      return 0;
    }
  }

  // Caps input at 0.9 or -0.9: for shooter wheels
  public double capInput(double input) {
    if (input > 0.9) {
      return 0.9;
    } else if (input < -0.9) {
      return -0.9;
    } else {
      return input;
    }
  }
  
  // Just a simple dead zone
  public double deadZoneComp(double x){
    if(Math.abs(x) < 0.08){
      return 0;
    }else{
      return x;
    }
  }

  public static double clicksToDegrees(double clicks) {
    return (int) (clicks / IntakeConstants.clicksPerDegree);
  }

  public static int degreesToClicks(double angle) {
    return (int) (IntakeConstants.CPR / 360 * angle);// Convert angle in degrees into usable clicks value
  }


/*
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


   //  int absolutePosition = intakeGearbox.getSensorCollection().getPulseWidthPosition();

     /* Mask out overflows, keep bottom 12 bits */
     /*absolutePosition &= 0xFFF;
     if (IntakeConstants.SENSOR_INVERT) { absolutePosition *= -1; }
     if (IntakeConstants.MOTOR_INVERT) { absolutePosition *= -1; }
     */
     /* Set the quadrature (relative) sensor to match absolute */
    // intakeGearbox.setSelectedSensorPosition(0, 0, IntakeConstants.TIMEOUT);
     
     //TODO: Force into starting position, then set baseline relative position? See the sample code:
     //https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/PositionClosedLoop/src/main/java/frc/robot/Robot.java
//}
 
/* double intakeHeading = 0;//Intake target
 int intakeIndexValue = 0;//Setpoints array address value

 public void intakePID() {
   
    //Intake PID Control
    // TODO: get new intake angles and put them in "Intake Constants"
    
    //Intake rotates to starting/highest position
    if(operatorController.getRawButton(4)) { // "Y" Button: starting position
      intakeHeading = IntakeConstants.SETPOINTS[0];
    }
    
    //Intake rotates to intaking/middle position
    if(operatorController.getRawButton(2)) { // "B" Button: intaking position
    intakeHeading = IntakeConstants.SETPOINTS[1];
    }
    
    //Intake rotates to scoring/stowed/folded all the way down position
    if(operatorController.getRawButton(1)) { // "A" button: stowed position
      intakeHeading = IntakeConstants.SETPOINTS[2];
    }

    //Move intake into the desired position
    intakeGearbox.set(ControlMode.Position,degreesToClicks(intakeHeading));
    
  } */
  

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
