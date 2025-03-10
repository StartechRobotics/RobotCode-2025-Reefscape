package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class grabber extends SubsystemBase {
  public final SparkMax m_grabber = new SparkMax(Constants.ID_GRABBER, MotorType.kBrushless);
  public SparkMaxConfig sparkConfig = new SparkMaxConfig();
  private MutVoltage grabVoltage = new MutVoltage(Constants.kGrabVolts, 1, Volts);
  private MutVoltage dropVoltage = new MutVoltage(Constants.kDropVolts, 1, Volts);
  public grabber() {
    sparkConfig.idleMode(IdleMode.kBrake);
    m_grabber.configure(sparkConfig, ResetMode.kResetSafeParameters ,PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

  }
  // --------- SIMPLE METHODS -------
  public void stopGrabber(){
    m_grabber.stopMotor();
  } 

  public void grab(){
    m_grabber.setVoltage(grabVoltage);
  }

  public void drop(){
    m_grabber.setVoltage(dropVoltage);
  }

  // -------- LAMBDA COMMANDS ------
  public Command grabCommand(grabber c_grabber){
    return Commands.runOnce(()-> c_grabber.grab(),c_grabber);
  }

  public Command dropCommand(grabber c_grabber){
    return Commands.runOnce(() -> c_grabber.drop(),c_grabber );
  }

  public Command stopCommand(grabber c_grabber){
    return Commands.runOnce(() -> c_grabber.stopGrabber(), c_grabber);
  }
}
