// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climber extends SubsystemBase {
  private final SparkMax m_climber = new SparkMax(Constants.ID_CLIMBER, MotorType.kBrushless);
  private SparkMaxConfig motorConfig = new SparkMaxConfig();
  private RelativeEncoder encoder = m_climber.getEncoder(); 
  private static final double forward = 1;
  private static final double reverse = -1;
   
  public climber() {
    motorConfig.idleMode(IdleMode.kBrake);
    m_climber.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed){
    m_climber.set(speed);
  }

  public void extend(){
    setSpeed(forward);
  }

  public void retract(){
    setSpeed(reverse);
  }

  public void stop(){
    setSpeed(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position", encoder.getPosition());
  }

  // Commands -----

  public Command stopCommand(){
    return Commands.runOnce(this::stop,this);
  }

  public Command extendCommand(){
    return new SequentialCommandGroup(
      Commands.runOnce(this::extend,this),
      Commands.waitUntil(()->this.encoder.getPosition() >= Constants.kExtendPosition),
      Commands.runOnce(this::stop,this)
      );
  }

  public Command retractCommand(){
    return new SequentialCommandGroup(
      Commands.runOnce(this::retract,this),
      Commands.waitUntil(()-> this.encoder.getPosition() <= Constants.kHangingPosition),
      stopCommand()
    );
  }

  public Command setNeutralCommand(){
    Command directionCommand = this.encoder.getPosition() > Constants.kNeutralPosition ? Commands.runOnce(this::retract, this) : Commands.runOnce(this::extend,this);
    return new SequentialCommandGroup(
      directionCommand,
      Commands.waitUntil(()->this.encoder.getPosition() >= Constants.kNeutralPosition),
      stopCommand()
    );
  }

}
