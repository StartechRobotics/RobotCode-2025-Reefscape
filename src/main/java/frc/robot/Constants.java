package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    // CHASSIS CONSTANTS
    public static final  int ID_IZQ_1 = 3;
    public static final int ID_IZQ_2 = 4;
    public static final int ID_DER_1 = 5;
    public static final int ID_DER_2 = 2;
    public static final double kSlowMode = 0.3;
    // ORIGINAL ENCODERS
    public static final int ID_ENC_1A = 9;
    public static final int ID_ENC_1B = 8;
    public static final int ID_ENC_2A = 7;
    public static final int ID_ENC_2B = 6;

    // PATH FOLLOWER
    public static final double kRatio_chassis = 1; // gearbox chassis ratio
    public static final double kTrackWitdth = 0.61; //61cm -> m
    public static final DifferentialDriveKinematics kDriveKinematics = 
        new DifferentialDriveKinematics(kTrackWitdth);
    public static final double MAX_SPEED_ms2 = 3;
    public static final double MAX_ACCEL_ms2 = 3;
    public static final double MAX_MOTOR_VOLTS = 10;
    public static final double MAX_ROTATION_SPEED_RAD_S = 3*Math.PI/2;
    public static final double kS_chassisLeft = 0;
    public static final double kV_chassisLeft = 0;
    public static final double kA_chassisLeft = 0;
    public static final double kS_chassisRight = 0;
    public static final double kV_chassisRight = 0;
    public static final double kA_chassisRight = 0;

    // IO - CONTROLLER
    public static final int ID_DRIVER_CHASSIS = 0;
    public static final int ID_DRIVER_MECH = 1;
    public static final int ID_TEST_CONTROLLER = 2;
    public static final int ID_JOYSTICK_ROT = 0;
    public static final int ID_JOYSTICK_SPEED = 3;
    public static final int ID_JOYSTICK_BRAKE = 2;

    // Driver Constants
    public static final double kRot = 1;
    public static final double kLoopRamp = 0.1;
    public static final double kDeadBandRot = 0.1;
    public static double kPIDThreshold = 0.5;
    
    // Grabber Constants
    public static final int ID_GRABBER = 19;
    public static final double kGrabVolts = 5;
    public static final double kDropVolts = -5;
    
    //Elevator Constants
    public static final int ID_ELEVATOR_IZQ = 8;
    public static final int ID_ELEVATOR_DER = 9;
    public static final double kP_elev = 0.02;
    public static final double kI_elev = 0;
    public static final double kD_elev = 0.0;
    public static final double kS_elev = 0;
    public static final double kG_elev = 0;
    public static final double kV_elev = 0;
    public static final double kA_elev = 0;
    public static final double MAX_ELEV_SPEED_PERCENT = 1;
    public static final double MAX_ELEV_VOLTS = 12*MAX_ELEV_SPEED_PERCENT;
    public static final double kElevDistancePerRotCM =  100* (56/1875) * (Math.PI * Units.inchesToMeters(1.29));
    public static final double kElevMaxHeight = 100;
    public static final double kDistancetoFloorInches = 26.48;
    public static final double kL1Position = 0;
    public static final double kL2Position = 36;
    public static final double kL3Position = 102;
    public static final double kL4Position = 102;
    public static final double kAlgae1Position = 0;
    public static final double kAlgae2Position = 0;

    // Shooter Constants
    public static final int ID_SHOOTER_BACK = 6;
    public static final int ID_SHOOTER_FRONT = 7;
    public static final double MAX_SHOOTER_VOLTS = 12*0.85;
    public static final int kShooterSensorThreshold = 0;
    public static final double kIntakeCurrentThreshold = 40;
    public static final int kProximityThreshold = 100;
    
    // Hopper Constants 
}
