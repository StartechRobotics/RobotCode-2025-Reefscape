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

  private final SparkMax master_neoL= new SparkMax(Constants.ID_ELEVATOR_IZQ,MotorType.kBrushless);
  private final SparkMax follower_neoR = new SparkMax(Constants.ID_ELEVATOR_DER, MotorType.kBrushless);
  private final SparkMaxConfig masterConfig = new SparkMaxConfig();
  private final SparkMaxConfig followerConfig = new SparkMaxConfig();
  private final RelativeEncoder masterEncoder= master_neoL.getEncoder();
  private final RelativeEncoder followEncoder= follower_neoR.getEncoder();
  private final boolean followerInverted = true;
  public final double distancePerRotation = Constants.kElevDistancePerRotMeters;
  private final PIDController pidController = new PIDController(Constants.kP_elev, Constants.kI_elev, Constants.kD_elev);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(Constants.kS_elev, Constants.kG_elev, Constants.kV_elev);

  public elevator() {
    masterConfig.idleMode(IdleMode.kBrake);
    followerConfig.idleMode(IdleMode.kBrake);
    followerConfig.follow(master_neoL,followerInverted);
    master_neoL.configure(masterConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    follower_neoR.configure(followerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);    
    SmartDashboard.putBoolean("Follower", follower_neoR.isFollower());
    resetEncoders();  
  }
  
  // -----------SYSID FOR FEEDFORWARD -------------------------------
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
  private final SysIdRoutine m_sysIdRoutine =
  new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          voltage -> {
            drive(voltage);
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism being
          // characterized.
          
          log -> {
            // Record a frame for the left motors.  Since these share an encoder, we consider
            // the entire group to be one motor.
            log.motor("elevator")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        master_neoL.get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(getDistanceMeters(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(getRateMetersPerSecond(masterEncoder), MetersPerSecond));
          },
          // Tell SysId to make generated commands require this subsystem, suffix test state in
          // WPILog with this subsystem's name ("elevator")
          this));
  public Command sysIdDynamic(SysIdRoutine.Direction direction, elevator elevator) {
    return m_sysIdRoutine.dynamic(direction);
  }
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction, elevator elevator) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  //  --------- SIMPLE FEEDBACK/ACTUATOR Methods ----------------------

  public void updateOutputLabels(){
    SmartDashboard.putNumber("Left Elev Encoder", masterEncoder.getPosition());
    SmartDashboard.putNumber("Right Elev Encoder", followEncoder.getPosition());
  }

  public void resetEncoders(){
    masterEncoder.setPosition(0);
    followEncoder.setPosition(0);
  }

  public void dryStop(){
    master_neoL.stopMotor();
  }

  public void drive(double drive_speed){
    if(Math.abs(drive_speed) > 1 ){
      drive_speed = drive_speed > 0 ? 1 : -1;
    }
    double motor_speed = drive_speed * Constants.MAX_ELEV_SPEED_PERCENT;
    master_neoL.set(motor_speed);
  }

  public void drive(Voltage drive_volts){
    MutVoltage motor_volts = drive_volts.mutableCopy();
    double voltsMagnitude = motor_volts.baseUnitMagnitude();
    if(Math.abs(voltsMagnitude)> Constants.MAX_ELEV_VOLTS){
      voltsMagnitude = voltsMagnitude>0 ? Constants.MAX_ELEV_VOLTS : -1* Constants.MAX_ELEV_VOLTS;
    }
    if (voltsMagnitude == 0){
      master_neoL.stopMotor();
    }else{
      motor_volts.mut_setMagnitude(voltsMagnitude);
      master_neoL.setVoltage(motor_volts);
    }
  }

  public void driveVoltsPercent(double voltsPercent){
    if(Math.abs(voltsPercent) > 1 ){
      voltsPercent = voltsPercent > 0 ? 1 : -1;
    }
    double motor_speed = voltsPercent * Constants.MAX_ELEV_SPEED_PERCENT;
    master_neoL.setVoltage(motor_speed*Constants.MAX_ELEV_VOLTS);
  }

  public void pidTarget(double setpointM) {
    double measurement = getDistanceMeters();
    double pidOutput = pidController.calculate(measurement, setpointM);
    // feedforward counteracts gravity and avoids subsystem not staying still
    double ffOutput = m_feedforward.calculate(0);
    SmartDashboard.putNumber("feedforward", ffOutput);
    double output = pidOutput + ffOutput;
    drive(output);
  }

  public void drive_PID_Volts(double setpoint) {
    double measurement = masterEncoder.getPosition();
    double pidOutput = pidController.calculate(measurement, setpoint);
    // feedforward counteracts gravity and avoids subsystem not staying still
    double ffOutput = m_feedforward.calculate(0);
    SmartDashboard.putNumber("feedforward", ffOutput);
    double output = pidOutput + ffOutput;
    driveVoltsPercent(output);
  }

  public double getDistanceMeters(){
    double distance = this.masterEncoder.getPosition() * distancePerRotation;
    return distance;
  }

  public double getRateMetersPerSecond(RelativeEncoder encoder){
    double rate_RotPerSecond = encoder.getVelocity()/60;
    double rate_MetersPerSecond = rate_RotPerSecond * distancePerRotation;
    return rate_MetersPerSecond;
  }

  // -------------PERIODIC-----------------

  @Override
  public void periodic() {
    updateOutputLabels();
  }

  // ------------ LAMBDA COMMANDS ---------------

  public Command driveCommand (elevator c_elevator, XboxController controller){
    return Commands.run(
      ()->c_elevator.drive(
        controller.getRawAxis(XboxController.Axis.kRightTrigger.value)
        -controller.getRawAxis(XboxController.Axis.kLeftTrigger.value))
      , c_elevator);
  }

  public Command driveToTargetCommand(double position, elevator c_elevator){
    return Commands.run(
      ()->
        c_elevator.pidTarget(position), 
        c_elevator)
      .until(() -> c_elevator.getDistanceMeters() == position)
      .andThen(
      Commands.runOnce(
        ()->
        c_elevator.dryStop()
      , c_elevator));
  }

  public Command dryStopCommand(elevator c_elevator ){
    return Commands.runOnce(()-> c_elevator.dryStop());
  }
}
