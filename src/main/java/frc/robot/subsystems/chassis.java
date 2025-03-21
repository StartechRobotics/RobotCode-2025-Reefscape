package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

public class chassis extends SubsystemBase {

    // class vars actuators-related
    private final WPI_TalonSRX izq_1 = new WPI_TalonSRX(Constants.ID_IZQ_1);
    private final WPI_TalonSRX izq_2 = new WPI_TalonSRX(Constants.ID_IZQ_2);
    private final WPI_TalonSRX der_1 = new WPI_TalonSRX(Constants.ID_DER_1);
    private final WPI_TalonSRX der_2 = new WPI_TalonSRX(Constants.ID_DER_2);
    // ** MotorcontrollerGroups reemplazados por Master (1) y Follower (2) por fallos en la caracterización del robot
    // ** Posiblemente regrese a configuracion Master-Slave si es necesario ahorrar recursos en el futuro
    private final MotorControllerGroup group_l = new MotorControllerGroup(izq_1, izq_2);
    private final MotorControllerGroup group_r = new MotorControllerGroup(der_1, der_2);

    // class vars odom-related
    private final DifferentialDrive differentialDrive = new DifferentialDrive(group_l,group_r);
    private final DifferentialDriveKinematics kinematics = Constants.kDriveKinematics;
    private DifferentialDriveOdometry m_odometry = null;
    private final SimpleMotorFeedforward lFeedforward = new SimpleMotorFeedforward(
      Constants.kS_chassisLeft,
      Constants.kV_chassisLeft,
      Constants.kA_chassisLeft
      );
    private final SimpleMotorFeedforward rFeedforward = new SimpleMotorFeedforward(
      Constants.kS_chassisRight,

      Constants.kV_chassisRight,
      Constants.kA_chassisRight
      );
    private Encoder l_encoder = null;
    private Encoder r_encoder = null;
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(); 
    // private final 
    public static final double kRot = Constants.kRot;
    public double speed_monitor;
    public double rot_monitor;
    private boolean odometry_engaged;
    private static final double encoder_dpp = (Math.PI * Units.inchesToMeters(6))/128;

    // class vars sysid-related
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
    
  public chassis() {
    setBrakeMode();
    group_r.setInverted(true);
    // izq_2.follow(izq_1);
    // der_2.follow(der_1);
    // izq_1.setInverted(true);
    // izq_2.setInverted(true);

    izq_1.configOpenloopRamp(Constants.kLoopRamp);
    izq_2.configOpenloopRamp(Constants.kLoopRamp);
    der_1.configOpenloopRamp(Constants.kLoopRamp);
    der_2.configOpenloopRamp(Constants.kLoopRamp);

    gyro.calibrate();

    try{ //FOR GPIO/DIO use try catch to avoid failure on robot Init
      l_encoder = new Encoder(Constants.ID_ENC_1A, Constants.ID_ENC_1B);
      r_encoder = new Encoder(Constants.ID_ENC_2A, Constants.ID_ENC_2B);
      l_encoder.reset();
      r_encoder.reset();
      l_encoder.setDistancePerPulse(encoder_dpp);
      r_encoder.setDistancePerPulse(encoder_dpp);
      l_encoder.setReverseDirection(false); 
      r_encoder.setReverseDirection(true);

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

    //PATHPLANNER LIB INITIALIZATION ------ (SNIPPET DE LA DOCUMENTACION OFICIAL PARA DIFFERENTIALDRIVES) 
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config; // Initialize config with a default value
    try{
      config = RobotConfig.fromGUISettings();
      System.out.println("Successfully Using PathPlanner GUI Robot Data");
    } catch (Exception e) {
      // Handle exception for missing GUI robot diagram
      e.printStackTrace();
      System.err.println("Failed to load RobotConfig from GUI settings. Using default configuration.");
      config = new RobotConfig(23, 0, new ModuleConfig(
        1, 
        1, 
        1, 
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
  }

  // End of constructor------------------

  @Override
  public void periodic() {
    updateOutputLabels();
    updateOdometry();
  }

  // SYSID-------------------------------

  // SNIPPET FROM WPILIB GITHUB EXAMPLE
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
                          group_l.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(l_encoder.getDistance(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(l_encoder.getRate(), MetersPerSecond));
              // Record a frame for the right motors.  Since these share an encoder, we consider
              // the entire group to be one motor.
              log.motor("drive-right")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          group_r.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(r_encoder.getDistance(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(r_encoder.getRate(), MetersPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test state in
            // WPILog with this subsystem's name ("drive")
            this));
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  // ------- REGULAR METHODS ------------

  public void setBrakeMode(){
    izq_1.setNeutralMode(NeutralMode.Brake);
    izq_2.setNeutralMode(NeutralMode.Brake);
    der_1.setNeutralMode(NeutralMode.Brake);
    der_2.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode(){
    izq_1.setNeutralMode(NeutralMode.Coast);
    izq_2.setNeutralMode(NeutralMode.Coast);
    der_1.setNeutralMode(NeutralMode.Coast);
    der_2.setNeutralMode(NeutralMode.Coast);
  }

  public void setMaxOutput(double maxOutput){
    differentialDrive.setMaxOutput(maxOutput);
  }

  public void clearFaults(){
    izq_1.clearStickyFaults();
    izq_2.clearStickyFaults();
    der_1.clearStickyFaults();
    der_2.clearStickyFaults();
    System.out.println("Successfully Cleared Drivetrain controllers Sticky Faults");
  }

  public void updateOutputLabels(){
    SmartDashboard.putNumber("LeftSpeed", izq_1.getMotorOutputPercent());
    SmartDashboard.putNumber("Left speed M/s", getLeftVelocity());
    SmartDashboard.putNumber("Right speed M/s", getRightVelocity());
    SmartDashboard.putNumber("RightSpeed", der_1.getMotorOutputPercent());
    SmartDashboard.putNumber("Vertical Speed", speed_monitor);
    SmartDashboard.putNumber("Current Rotation", rot_monitor);
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


  // ------------GETTER METHODS ----------------

  public ADXRS450_Gyro getGyro(){
    return gyro;
  } 

  public Encoder getRightEncoder(){
    return r_encoder;
  }

  public Encoder getLeftEncoder(){
    return l_encoder;
  }

  // -----------ODOM METHODS --------------------

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
    m_odometry.resetPosition(gyro.getRotation2d(), l_encoder.getDistance(), r_encoder.getDistance(), pose);
  }

  public double getRightVelocity(){
    return r_encoder.getRate();
  }

  public double getLeftVelocity(){
    return l_encoder.getRate();
  }

  public double getTurnRate(){
    return gyro.getRate();
  }

  public void updateOdometry(){
    speed_monitor = getAverageEncoderSpeed();
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
    // group_l.setVoltage(lFeedforward.calculate(l_volts));
    // group_r.setVoltage(rFeedforward.calculate(r_volts));
    group_l.setVoltage(l_volts);
    group_r.setVoltage(r_volts);
    differentialDrive.feed();
  }

  public void setMotorVolts(Voltage l_volts, Voltage r_volts){
    group_l.setVoltage(lFeedforward.calculate(l_volts.magnitude()));
    group_r.setVoltage(rFeedforward.calculate(r_volts.magnitude()));
    differentialDrive.feed();
  }

  public void drive(ChassisSpeeds chassisSpeeds){
    double forwardXSpeed = chassisSpeeds.vxMetersPerSecond;
    if (Math.abs(forwardXSpeed) > Constants.MAX_SPEED_ms2) {
      forwardXSpeed = forwardXSpeed > 0 ? Constants.MAX_SPEED_ms2 : -Constants.MAX_SPEED_ms2;
    }
    double angularVelocity = chassisSpeeds.omegaRadiansPerSecond;
    rot_monitor = angularVelocity;
    double leftVelocity = forwardXSpeed + (angularVelocity * Constants.kTrackWitdth / 2);
    double rightVelocity = forwardXSpeed - (angularVelocity * Constants.kTrackWitdth / 2);

    double leftVolts = Constants.MAX_MOTOR_VOLTS*(leftVelocity/Constants.MAX_SPEED_ms2);
    double rightVolts = Constants.MAX_MOTOR_VOLTS*(rightVelocity/Constants.MAX_SPEED_ms2);
    setMotorVolts(leftVolts, rightVolts);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    drive(chassisSpeeds);
  }

  public void arcadeDrive(double speed, double rot, boolean slowMode){
    // deadbands
    rot = Math.abs(rot) >= 0.001 ? rot : 0;
    speed = Math.abs(speed) >= 0.001 ? speed :0;
    speed = slowMode ? speed*Constants.kSlowMode : speed;
    rot = slowMode ? rot*(Constants.kSlowMode*1.75) : rot;
    double forwardSpeed = speed*Constants.MAX_SPEED_ms2;
    double rotationSpeed = rot*Constants.MAX_ROTATION_SPEED_RAD_S;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      forwardSpeed, 
      0, 
      rotationSpeed
    );
    drive(chassisSpeeds);
  }

  public boolean StopChassis(){
    group_l.stopMotor();
    group_r.stopMotor();
    // Followers stop automatically
    return true;
  }

  @Deprecated(forRemoval = true)
  public void setOrientationAngle(double target){
    double current_theta = gyro.getAngle();
    double error = target - current_theta;
    double turn = 0; // Initialize turn value
    double Kp = 0.1; // Proportional gain
    double Ki = 0.01; // Integral gain
    double Kd = 0.01; // Derivative gain
    double S_theta = 0;
    double previous_error = 0;
  
    while (!Thread.interrupted()) {
      current_theta = gyro.getAngle();
      error = target - current_theta;
      SmartDashboard.putNumber("Gyro Error", error);
  
      double P = Kp * error; 
      S_theta += error;
      double I = Ki * S_theta;
      double d_theta = error - previous_error;
      double D = Kd * d_theta;

      turn = P + I + D;
      turn = Math.min(Math.max(turn, -0.7), 0.7);
  
      // arcadeDrive(0, turn);

      previous_error = error;
  
      if(Math.abs(error) < Constants.kPIDThreshold){
        StopChassis();
        SmartDashboard.putNumber("Gyro Error", 0);
        break;
      }
    }
  }

// CHASSIS LAMBDA COMMANDS -----------------------

  
  public Command driveCommand(XboxController controller){ 
    
    return Commands.run(
      () -> 
        this.arcadeDrive(
        (controller.getLeftBumperButton()? 0.4 : 1)*
        (controller.getRawAxis(XboxController.Axis.kLeftTrigger.value)- controller.getRawAxis(XboxController.Axis.kRightTrigger.value)),
        (Math.abs(controller.getRawAxis(Constants.ID_JOYSTICK_ROT)) > Constants.kDeadBandRot ? controller.getRawAxis(Constants.ID_JOYSTICK_ROT) : 0),
        controller.getLeftBumper()), 
      this);
  }

  public Command resetGyroCommand(){
    return this.runOnce(this.gyro::reset);
  }

  @Deprecated
  public Command setOrientationCommand(double theta){
    return Commands.run(()-> this.setOrientationAngle(theta), this);
  }

  public Command resetOdometryCommand(){
    return this.runOnce(this::resetOdometry).ignoringDisable(true);
  }

  public Command stopCommand(){
    return this.runOnce(this::StopChassis);
  }

  public Command clearFaultsCommand(){
    return this.runOnce(this::clearFaults);
  }
  
}