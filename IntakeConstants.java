package frc.robot;

public class IntakeConstants {
    //Sensor
  static final boolean MOTOR_INVERT = false;//if motor moves in wrong direction, flip this
  //TODO: Set the correct values for these
  
  static final int GEARBOX_SPROCKET_TEETH = 16;
  static final int ARM_SPROCKET_TEETH = 42;
  static final double SPROCKET_RATIO = ARM_SPROCKET_TEETH / GEARBOX_SPROCKET_TEETH;
  static final double CPR = 4096 * SPROCKET_RATIO;//clicks per rotation adjusted for sprocket reduction
  static final boolean SENSOR_INVERT = false;//if sensor outputs are wrong, flip this
  static final int PID_SLOT = 0;
  static final int TIMEOUT = 30;
  static final double kF = 0.0;
  static final double kP = 0.0;
  static final double kI = 0.0;
  static final double kD = 0.0;

  //Intake position set points (degrees)
  static int[] SETPOINTS ={
      0,//starting config
      49,//intaking postition
      79//Scoring config: tucked against bumpers
  };

  static String[] SETPOINT_NAMES ={
    "Starting Config",
    "Intaking",
    "Scoring"
  };
}
