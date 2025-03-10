// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class SysID_chassis extends SubsystemBase {

  private final WPI_TalonSRX izq_1 = new WPI_TalonSRX(Constants.ID_IZQ_1);
    private final WPI_TalonSRX izq_2 = new WPI_TalonSRX(Constants.ID_IZQ_2);
    private final WPI_TalonSRX der_1 = new WPI_TalonSRX(Constants.ID_DER_1);
    private final WPI_TalonSRX der_2 = new WPI_TalonSRX(Constants.ID_DER_2);

    private final DifferentialDrive differentialDrive = new DifferentialDrive(izq_1, izq_2);
    private final DifferentialDriveKinematics kinematics = Constants.kDriveKinematics;
    private DifferentialDriveOdometry m_odometry;
    private Encoder l_encoder;
    private Encoder r_encoder;
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(); 
    public Timer timer = new Timer();
    public static final double kRot = Constants.kRot;
    public static double speed_monitor;
    public static double rot_monitor;
    public XboxController driver_controller;
    public double last_encoder_R = 0;
    public double last_encoder_L = 0;
    public double last_timer;
    public boolean odometry_engaged;
    public static final double encoder_dpp = (Math.PI * Units.inchesToMeters(6) / Constants.kRatio_chassis)/128;

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  /** Creates a new SysID_chassis. */
  public SysID_chassis() {
    izq_2.follow(izq_1);
    der_2.follow(der_1);
    izq_1.setInverted(true);
    izq_2.setInverted(true);

    izq_1.configOpenloopRamp(Constants.kLoopRamp);
    izq_2.configOpenloopRamp(Constants.kLoopRamp);
    der_1.configOpenloopRamp(Constants.kLoopRamp);
    der_2.configOpenloopRamp(Constants.kLoopRamp);

    gyro.calibrate();

    try{
      l_encoder = new Encoder(Constants.ID_ENC_1A, Constants.ID_ENC_1B);
      r_encoder = new Encoder(Constants.ID_ENC_2A, Constants.ID_ENC_2B);
      l_encoder.reset();
      r_encoder.reset();
      l_encoder.setDistancePerPulse(encoder_dpp);
      r_encoder.setDistancePerPulse(encoder_dpp); 

      m_odometry = new DifferentialDriveOdometry(
        gyro.getRotation2d(), 
        l_encoder.getDistance(), 
        r_encoder.getDistance()
      );
      
      m_odometry.resetPosition(
        gyro.getRotation2d(), 
        l_encoder.getDistance(), 
        r_encoder.getDistance(), 
        new Pose2d()
      );

      odometry_engaged = true;

    }catch(Exception e){
      odometry_engaged=false;
    }finally{
      SmartDashboard.putBoolean("Encoder Status", odometry_engaged);
    }

    RobotConfig config; // Initialize config with a default value
    try{
      config = RobotConfig.fromGUISettings();
      System.out.println("Successfully Using PathPlanner GUI Robot Data");
    } catch (Exception e) {
      // Handle exception for missing GUI robot diagram
      e.printStackTrace();
      System.err.println("Failed to load RobotConfig from GUI settings. Using default configuration.");
      config = new RobotConfig(78, 6.883, new ModuleConfig(
        0.048, 
        5.450, 
        1.200, 
        DCMotor.getCIM(1), 
        60, 
        4));
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements .... fin del SNIPPET
    );

    resetOdometry();
    gyro.reset();
    timer.start();
  }

  @Override
  public void periodic() {
    updateOutputLabels();
    updateOdometry();
  }

  // SYSID BUILD SNIPPET
private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            voltage -> {
              setMotorVolts(voltage, voltage);
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism being
            // characterized.
            
            log -> {
              // Record a frame for the left motors.  Since these share an encoder, we consider
              // the entire group to be one motor.
              log.motor("drive-left")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          izq_1.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(l_encoder.getDistance(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(l_encoder.getRate(), MetersPerSecond));
              // Record a frame for the right motors.  Since these share an encoder, we consider
              // the entire group to be one motor.
              log.motor("drive-right")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          der_1.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(r_encoder.getDistance(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(r_encoder.getRate(), MetersPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test state in
            // WPILog with this subsystem's name ("drive")
            this));
  public Command sysIdDynamic(SysIdRoutine.Direction direction, SysID_chassis c_chassis) {
    return m_sysIdRoutine.dynamic(direction);
  }
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction, SysID_chassis c_chassis) {
    return m_sysIdRoutine.quasistatic(direction);
  }
  // Methods

  public void clearFaults(){
    izq_1.clearStickyFaults();
    izq_2.clearStickyFaults();
    der_1.clearStickyFaults();
    der_2.clearStickyFaults();
    System.out.println("Successfully Cleared Drivetrain controllers Sticky Faults");
  }
  public void updateOutputLabels(){
    SmartDashboard.putNumber("LeftSpeed", izq_1.getMotorOutputPercent());
    SmartDashboard.putNumber("RightSpeed", der_1.getMotorOutputPercent());
    
    SmartDashboard.putBoolean("Motor5 status", izq_1.isAlive());
    SmartDashboard.putBoolean("Motor2 status", izq_2.isAlive());
    SmartDashboard.putBoolean("Motor3 status", der_1.isAlive());
    SmartDashboard.putBoolean("Motor4 status", der_2.isAlive());
    SmartDashboard.putNumber("Gyro", gyro.getAngle());
    SmartDashboard.putNumber("Left Encoder", l_encoder.getDistance());
    SmartDashboard.putNumber("Right Encoder", r_encoder.getDistance());
    SmartDashboard.putNumber("izq 1 Volts", izq_1.get());
    SmartDashboard.putNumber("izq 2 Volts", izq_2.get());
    SmartDashboard.putNumber("der 1 Volts", der_1.get());
    SmartDashboard.putNumber("der 2 Volts", der_2.get());
  }

  public ADXRS450_Gyro getGyro(){
    return gyro;
  } 

  public Encoder getRightEncoder(){
    return r_encoder;
  }

  public Encoder getLeftEncoder(){
    return l_encoder;
  }

  public void resetEncoders(){
    l_encoder.reset();
    r_encoder.reset();
  }
  
  public void resetOdometry(){
    l_encoder.reset();
    r_encoder.reset();
    m_odometry.resetPosition(
        gyro.getRotation2d(), 
        l_encoder.getDistance(), 
        r_encoder.getDistance(), 
        new Pose2d()
      );
  }

  public void resetPose(Pose2d pose) {
    // Reset the odometry with the new pose and current gyro angle
    m_odometry.resetPosition(gyro.getRotation2d(), l_encoder.getDistance(), r_encoder.getDistance(), pose);
}

  public double getRightVelocity(){
    double d_distance = r_encoder.getDistance() - last_encoder_R;
    double d_time = timer.get() - last_timer;
    double velocity = d_time > 0 ? d_distance/d_time : 0;
    return r_encoder.getRate();
  }

  public double getLeftVelocity(){
    double d_distance = -(l_encoder.getDistance() - last_encoder_L);
    double d_time = timer.get() - last_timer;
    double velocity = d_time > 0 ? d_distance/d_time : 0;
    return l_encoder.getRate();
  }

  public double getTurnRate(){
    return gyro.getRate();
  }

  public void updateOdometry(){
    last_encoder_L = l_encoder.getDistance();
    last_encoder_R = r_encoder.getDistance();
    last_timer = timer.get();
    m_odometry.update(gyro.getRotation2d(), l_encoder.getDistance(),r_encoder.getDistance());
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public double getAverageEncoderSpeed(){
    return (getLeftVelocity()+getRightVelocity())/2;
  }

  public double getAverageEncoderDistance(){
    return (l_encoder.getDistance()+r_encoder.getDistance())/2;
  }

  public ChassisSpeeds getCurrentSpeeds(){
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  // ----------MOVEMENT METHODS ------------------

  public void setMotorVolts(double l_volts, double r_volts){
    izq_1.setVoltage(l_volts);
    der_1.setVoltage(r_volts);
    differentialDrive.feed();
  }

  public void setMotorVolts(Voltage l_volts, Voltage r_volts){
    izq_1.setVoltage(l_volts);
    der_1.setVoltage(r_volts);
    differentialDrive.feed();
  }

  public void drive(ChassisSpeeds chassisSpeeds){
    // ----PREVIOUS alg underperformed
    // DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    // differentialDrive.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    // differentialDrive.feed();

    double forwardXSpeed = chassisSpeeds.vxMetersPerSecond;
    if(Math.abs(forwardXSpeed)>Constants.MAX_SPEED_ms2){
      forwardXSpeed = forwardXSpeed > 0 ? Constants.MAX_SPEED_ms2 : -Constants.MAX_SPEED_ms2;
    }

    double angularVelocity = chassisSpeeds.omegaRadiansPerSecond;

    double leftVelocity = forwardXSpeed - (angularVelocity * Constants.kTrackWitdth / 2);
    double rightVelocity = forwardXSpeed + (angularVelocity * Constants.kTrackWitdth / 2);

    double leftVolts = Constants.MAX_MOTOR_VOLTS*(leftVelocity/Constants.MAX_SPEED_ms2);
    double rightVolts = Constants.MAX_MOTOR_VOLTS*(rightVelocity/Constants.MAX_SPEED_ms2);
    setMotorVolts(leftVolts, rightVolts);
    differentialDrive.feed();
    // differentialDrive.feedWatchdog();
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    drive(chassisSpeeds);
  }

  public boolean StopChassis(){
    izq_1.stopMotor();
    der_1.stopMotor();
    // Followers stop automatically
    return true;
  }

  public Command resetGyroCommand(SysID_chassis c_chassis){
    return this.runOnce(() -> c_chassis.getGyro().reset());
  }

  public Command resetOdometryCommand(SysID_chassis c_chassis){
    return this.runOnce(()-> c_chassis.resetOdometry());
  }

  public Command stopCommand(SysID_chassis c_Chassis){
    return this.runOnce(() -> c_Chassis.StopChassis());
  }

  public Command clearFaultsCommand(SysID_chassis c_chassis){
    return this.runOnce(() -> c_chassis.clearFaults());
  }
}
