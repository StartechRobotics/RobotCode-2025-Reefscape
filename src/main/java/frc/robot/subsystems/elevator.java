// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class elevator extends SubsystemBase {

  private final SparkMax master_neoL = new SparkMax(Constants.ID_ELEVATOR_DER, MotorType.kBrushless);
  private final SparkMax follower_neoR = new SparkMax(Constants.ID_ELEVATOR_IZQ, MotorType.kBrushless);
  private final SparkMaxConfig masterConfig = new SparkMaxConfig();
  private final SparkMaxConfig followerConfig = new SparkMaxConfig();
  private final RelativeEncoder masterEncoder = master_neoL.getEncoder();
  private final RelativeEncoder followEncoder = follower_neoR.getEncoder();
  private final boolean masterInverted = false;
  private final boolean followerInverted = !masterInverted;
  public static final double distancePerRotation = Constants.kElevDistancePerRotCM;
  private final PIDController pidController = new PIDController(
    Constants.kP_elev, 
    Constants.kI_elev,
    Constants.kD_elev);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
    Constants.kS_elev, 
    Constants.kG_elev,
    Constants.kV_elev,
    Constants.kA_elev);
  public int position = 1;
  public double displacement=0;


  public elevator() {
    masterConfig.idleMode(IdleMode.kBrake);
    followerConfig.idleMode(IdleMode.kBrake);
    masterConfig.inverted(masterInverted);
    followerConfig.follow(master_neoL, followerInverted);
    master_neoL.configure(masterConfig, SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);
    follower_neoR.configure(followerConfig, SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);
    SmartDashboard.putBoolean("Follower", follower_neoR.isFollower());
    resetEncoders();
  }

  // -----------SYSID FOR FEEDFORWARD -------------------------------

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          voltage -> {
            drive(voltage);
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.

          log -> {
            // Record a frame for the left motors. Since these share an encoder, we consider
            // the entire group to be one motor.
            log.motor("elevator left Master")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        master_neoL.get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(getDistanceMetersMaster(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(getRateMetersPerSecond(masterEncoder), MetersPerSecond));
            log.motor("elevator right Follower")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        follower_neoR.get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(getDistanceMetersFollower(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(getRateMetersPerSecond(followEncoder), MetersPerSecond));
          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in
          // WPILog with this subsystem's name ("elevator")
          this));

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  // --------- SIMPLE FEEDBACK/ACTUATOR Methods ----------------------

  public void updateOutputLabels() {
    SmartDashboard.putNumber("Left Elev Encoder", masterEncoder.getPosition());
    SmartDashboard.putNumber("Right Elev Encoder", followEncoder.getPosition());
    SmartDashboard.putNumber("Height", displacement);
    SmartDashboard.putNumber("Left Temp", getLeftTemp());
    SmartDashboard.putNumber("Right Temp", getRightTemp());
    SmartDashboard.putNumber("L Position", position);
    SmartDashboard.putData(null);
  }

  public void resetEncoders() {
    masterEncoder.setPosition(0);
    followEncoder.setPosition(0);
  }

  public void dryStop() {
    master_neoL.stopMotor();
  }

  public void drive(double drive_speed) {
    if (Math.abs(drive_speed) > 1) {
      drive_speed = drive_speed > 0 ? 1 : -1;
    }
    double motor_speed = drive_speed * Constants.MAX_ELEV_SPEED_PERCENT;
    motor_speed = stopAtLimits(motor_speed);
    master_neoL.set(motor_speed);
  }

  public void drive(Voltage drive_volts) {
    MutVoltage motor_volts = drive_volts.mutableCopy();
    double voltsMagnitude = motor_volts.baseUnitMagnitude();

    if (Math.abs(voltsMagnitude) > Constants.MAX_ELEV_VOLTS) {
      voltsMagnitude = voltsMagnitude > 0 ? Constants.MAX_ELEV_VOLTS : -1 * Constants.MAX_ELEV_VOLTS;
    }

    voltsMagnitude = stopAtLimits(voltsMagnitude);
    if(voltsMagnitude == 0){
      dryStop();
    }else{
      motor_volts.mut_setMagnitude(voltsMagnitude);
      master_neoL.setVoltage(motor_volts);
    }
  }

  public void driveVoltsPercent(double voltsPercent) {
    if (Math.abs(voltsPercent) > 1) {
      voltsPercent = voltsPercent > 0 ? 1 : -1;
    }
    double motor_speed = voltsPercent * Constants.MAX_ELEV_SPEED_PERCENT;
    master_neoL.setVoltage(motor_speed * Constants.MAX_ELEV_VOLTS);
  }

  public void pidTarget(double setpointM) {
    double measurement = masterEncoder.getPosition();
    double pidOutput = pidController.calculate(measurement, setpointM);
    // feedforward counteracts gravity and avoids subsystem not staying still
    double ffOutput = 0;
    // double ffOutput = m_feedforward.calculate(0);
    // SmartDashboard.putNumber("feedforward", ffOutput);
    double output = pidOutput + ffOutput;
    drive(output);
  }

  public void drive_PID_Volts(double setpoint) {
    double measurement = masterEncoder.getPosition();
    double pidOutput = pidController.calculate(measurement, setpoint);
    // feedforward counteracts gravity and avoids subsystem not staying still
    SmartDashboard.putNumber("Error", setpoint-measurement);
    SmartDashboard.putNumber("PID Out", pidOutput);
    double ffOutput = m_feedforward.calculate(0);
    SmartDashboard.putNumber("feedforward", ffOutput);
    double output = pidOutput + ffOutput;
    driveVoltsPercent(output);
  }

  public double getDistanceMetersMaster() {
    double distance = this.masterEncoder.getPosition() * distancePerRotation / 100;
    return distance;
  }

  public double getDistanceMetersFollower() {
    double distance = this.followEncoder.getPosition() * distancePerRotation / 100;
    return distance;
  }

  public double getDistanceMeters(){
    return getDistanceMetersMaster();
  }

  public double getDistanceCentimeters() {
    return masterEncoder.getPosition() * distancePerRotation;
  }

  public double getRateMetersPerSecond(RelativeEncoder encoder) {
    double rate_RotPerSecond = encoder.getVelocity() / 60;
    double rate_MetersPerSecond = rate_RotPerSecond * distancePerRotation / 100;
    return rate_MetersPerSecond;
  }

  public double getRateCentimetersPerSecond(RelativeEncoder encoder) {
    double rate_RotPerSecond = encoder.getVelocity() / 60;
    double rate_CentimetersPerSecond = rate_RotPerSecond * distancePerRotation;
    return rate_CentimetersPerSecond;
  }

  // --------- CONTROL METHODS -----------

  public double getRightTemp() {
    return follower_neoR.getMotorTemperature();
  }

  public double getLeftTemp() {
    return master_neoL.getMotorTemperature();
  }

  private double stopAtLimits(double input) {
    double output = input;
    if (this.masterEncoder.getPosition() <= 0 && input < 0) {
      output = 0;
    }
    if(this.masterEncoder.getPosition() >= 187 && input > 0){
      output = 0;
    }
    return output;
  }

  // -------------PERIODIC-----------------

  @Override
  public void periodic() {
    updateOutputLabels();
    displacement=masterEncoder.getPosition()*distancePerRotation;
  }

  // ------------ LAMBDA COMMANDS ---------------

  public Command driveCommand(XboxController controller) {
    return Commands.run(
        () -> this.drive(
            0.5*(controller.getRawAxis(XboxController.Axis.kRightTrigger.value)
                - controller.getRawAxis(XboxController.Axis.kLeftTrigger.value))),
        this);
  }

  public Command driveToTargetCommand(double position) {
    return Commands.run(
        () -> this.pidTarget(position),
        this)
        .beforeStarting(
          ()-> {
            if(position == Constants.kL1Position)this.position=1;
            else if(position == Constants.kL2Position)this.position=2;
            else if(position == Constants.kL3Position)this.position=3;
            else if(position == Constants.kL4Position)this.position=4;
          })
        .until(
          () -> masterEncoder.getPosition() == position)
        .andThen(
          Commands.runOnce(
            () -> this.dryStop()
        ,this));
  }

  public Command dryStopCommand() {
    return Commands.runOnce(() -> this.dryStop());
  }

}
