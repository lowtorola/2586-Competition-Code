package frc.robot;

public class ElevatorConstants {

    static final boolean MOTOR_INVERT = false;//if motor moves in wrong direction, flip this
    static final int WINCH_ID = 5;//change this to the correct talon ID.

    //Sensor
    static final int CPR = 4096;//clicks per rotation
    static final boolean SENSOR_INVERT = false;//if sensor outputs are wrong, flip this
    static final int PID_SLOT = 0;
    static final int TIMEOUT = 30;
    static final double kF = 0.0;
    static final double kP = 0.0;
    static final double kI = 0.0;
    static final double kD = 0.0;

    //Game Piece characteristics
    static final double HP_RADIUS = 8;//inches
    static final double CARGO_RADIUS = 6.5;//inches

    //Field Characteristics (Relative to ground)
    static final double gLOW_HATCH_HEIGHT = 19;//to center
    static final double gROCKET_PORT1 = 27.5;//to center
    static final double gCARGOSHIP_PORT = 31.5 + CARGO_RADIUS + 1;// ball has to clear this, 1 inch extra
    static final double ROCKET_HEIGHTCHANGE = 28.0;//center to center between rocket goals

    //Lift characteristics
    static final double CARGO_MECH_BOTTOM = 7;//from ground to center of mech in lift starting position
    static final double HP_MECH_BOTTOM = 15;//from ground to center of mech in lift starting position
    static final double DRUM_DIAMETER = 2.1;//drum of winch
    static final double DRUM_CIRCUMFERENCE = Math.PI * DRUM_DIAMETER;//inches

    //Field Characteristics (Relative to mechanisms in lift starting position)
    static final double mLOW_HATCH_HEIGHT = gLOW_HATCH_HEIGHT - HP_MECH_BOTTOM;
    static final double mROCKET_PORT1 = gROCKET_PORT1 - CARGO_MECH_BOTTOM;
    static final double mCARGOSHIP_PORT = gCARGOSHIP_PORT - CARGO_MECH_BOTTOM;

    //String array of setpoint names
    static final String[] SETPOINT_NAMES ={
    "Starting Config",
    "Low Hatch",
    "Rocket Port 1",
    "Cargo Ship Port",
    "Rocket Hatch 2",
    "Rocket Port 2",
    "Rocket Hatch 3",
    "Rocket Port 3"
    };

    //Lift Setpoints Array in height order (bottom to top)
    static final double[] SETPOINTS = {
        0.0,//bottom
        mLOW_HATCH_HEIGHT,
        mROCKET_PORT1,
        mCARGOSHIP_PORT,
        (mLOW_HATCH_HEIGHT + ROCKET_HEIGHTCHANGE),//Rocket Hatch 2
        (mROCKET_PORT1 + ROCKET_HEIGHTCHANGE),//Rocket Port 2
        (mLOW_HATCH_HEIGHT + (ROCKET_HEIGHTCHANGE * 2)),//Rocket Hatch 3
        (mROCKET_PORT1 + (ROCKET_HEIGHTCHANGE * 2))//Rocket Port 3
    };
}