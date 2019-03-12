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

  // Elevator elevatorWinch
  WPI_TalonSRX elevatorWinch;
  DigitalInput mainElevatorLow, mainElevatorHigh; // lift limit switches
	boolean mainElevatorIsNotLow;
	boolean mainElevatorIsNotHigh;

  // Shooter
  WPI_TalonSRX shooter;
  DigitalInput shooterLimit;
  boolean hasCargo; // To tell the operator we have a ball: based on the limit switch

  // Intake
  WPI_TalonSRX intakeGearbox;
  PWMVictorSPX intakeWheels;

  // HP Mech
  DoubleSolenoid mechUpDown, hatchGrabRelease;
  boolean hasHatch = false; // To tell the operator we have a hatch
  boolean hatchMechUp = true; // To print the hatch mech's position to SB
  Compressor comp;

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
   camera.setResolution(200, 150);
   camera.setFPS(30);

    // Elevator winch
    elevatorWinch = new WPI_TalonSRX(5);
    mainElevatorLow = new DigitalInput(3);
    mainElevatorHigh = new DigitalInput(4);
    mainElevatorIsNotLow = mainElevatorLow.get(); // low limit switch on lift value
    mainElevatorIsNotHigh = mainElevatorHigh.get(); // high limit switch on lift value

    // Shooter
    shooter = new WPI_TalonSRX(6);
    shooterLimit = new DigitalInput(0); // Limit switch to prevent from ripping cargo
    hasCargo = !shooterLimit.get(); // Boolean to use flip limit switch's input and use it

    // Intake
    intakeGearbox = new WPI_TalonSRX(7);
    intakeWheels = new PWMVictorSPX(0);
    intakeInit(); // Method to set up intake gearbox: set it in its own method for organization

    // HP Mech
    mechUpDown = new DoubleSolenoid(0, 1); // Solenoid to rotate the HP Mech up and down
    hatchGrabRelease = new DoubleSolenoid(2, 3); // Solenoid to grab and release HP
    comp = new Compressor();
    comp.start();

  }
  int counter = 0; // Integer to make it so that we print out to ShuffleBoard every 10 cycles
  @Override
  public void robotPeriodic() {
    counter = (counter + 1) % 10; // Increase counter by 1, get the remainder to use later
    if(counter == 0) {
    SmartDashboard.putBoolean("Has Hatch Panel", hasHatch); // Print out that we have a HP
    SmartDashboard.putBoolean("Has Cargo", hasCargo); // Print out that we have a Cargo
    if(hatchMechUp) {
      SmartDashboard.putString("Hatch Mech Position", "Up"); // Print out that the HP mech is up
    } else {
      SmartDashboard.putString("Hatch Mech Position", "Down"); // Print out that the HP mech is up
    }
    SmartDashboard.putString("Intake Position", IntakeConstants.SETPOINT_NAMES[intakeIndexValue]); // Print out the name of our intake target point
    if(!mainElevatorIsNotHigh) {
      SmartDashboard.putString("Lift Position", "Top");
    } else if(!mainElevatorIsNotLow) {
      SmartDashboard.putString("Lift Position", "Bottom");
    } else {
      SmartDashboard.putString("Lift Position", "Floating");
    }
    SmartDashboard.putNumber("lY", leftJoyStick.getY());
    SmartDashboard.putNumber("rY", rightJoyStick.getY());
    SmartDashboard.putNumber("Lift Direction", operatorController.getRawAxis(1));

    if(rightJoyStick.getRawButton(11)) { // Vision drive
      SmartDashboard.putString("Drive Mode", "Vision Mode"); 
    } else if(leftJoyStick.getRawButton(6)) { // Creep drive
      SmartDashboard.putString("Drive Mode", "Scoring Drive");
    } else { // Normal tank drive
     SmartDashboard.putString("Drive Mode", "Tank Drive");
    }
    SmartDashboard.putNumber("Intake Encoder", intakeGearbox.getSelectedSensorPosition());
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
    auxilaries(); // Method for controlling mechs
    leftPower = deadzoneComp(leftJoyStick.getY()) * -1;
    rightPower = deadzoneComp(rightJoyStick.getY()) * -1; 

    if(rightJoyStick.getRawButton(11)) { // Vision drive
      visionLogic(); // Vision processing
      SmartDashboard.putString("Auton Drive", "Robot is in Control"); 
    } else if(leftJoyStick.getRawButton(6)) { // Creep drive
      scoringDrive();
      SmartDashboard.putString("Scoring Drive", "Manual Scoring");
    } else { // Normal tank drive
      mainDrive.tankDrive(leftPower, rightPower);
     SmartDashboard.putString("Open Field Drive", "Regular");
    }
  }

  public void auxilaries(){ // Method containing all mechs except for elevator and intake
    
    double liftCommand = operatorController.getRawAxis(1); // Left stick: -1 is forward
		if (mainElevatorIsNotLow && liftCommand < 0) { // go down if we're not already at the bottom
			elevatorWinch.set(liftCommand);
		} else if (mainElevatorIsNotHigh && liftCommand > 0) { // go up if we're not already at the top
			elevatorWinch.set(liftCommand);
		} else {
			elevatorWinch.set(0);
		}

    double shooterCommand = operatorController.getRawAxis(0); // TODO: Make sure limit override is correct buttton and shooter is correct axis (left stick X axis)
		if(hasCargo || operatorController.getRawButton(3)) { // if we don't have cargo, make cargo intakable (or by l.s. override)
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
  
  //Intake: Controlled by PID method
  

  // Intake wheels- controlled by Right Trigger
  // We won't ever need to outtake from the bar intake
  double intakeWheelsControl = deadzoneComp(operatorController.getTriggerAxis(GenericHID.Hand.kRight) * -1);
  intakeWheels.set(intakeWheelsControl);
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


  public static int degreesToClicks(double angle) {
    return (int) (IntakeConstants.CPR / 360 * angle);// Convert angle in degrees into usable clicks value
  }



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
 
 double intakeHeading = 0;//Intake target
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
    
/*
    //For initial testing
    intakeGearbox.set(ControlMode.PercentOutput, operatorController.getRawAxis(1));//figure out motor direction
    intakeGearbox.getSensorCollection().getPulseWidthPosition();//figure out sensor direction
  */
  }
  

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
