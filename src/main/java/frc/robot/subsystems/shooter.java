package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.BooleanSupplier;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class shooter extends SubsystemBase {
  private static final int kProximityThreshold = Constants.kProximityThreshold;
  private static final double kCurrentThreshold = Constants.kIntakeCurrentThreshold;
  private final SparkMax m_front = new SparkMax(Constants.ID_SHOOTER_FRONT, MotorType.kBrushless);
  private final SparkMax m_back = new SparkMax(Constants.ID_SHOOTER_BACK, MotorType.kBrushless);
  private SparkMaxConfig backConfig = new SparkMaxConfig();
  private SparkMaxConfig frontConfig = new SparkMaxConfig();
  private LinearFilter lowpassFilter = LinearFilter.singlePoleIIR(0.02, 0.02);
  public boolean coralIsFront;
  
  // private static final MutVoltage intakeVolts = new MutVoltage(8.5, 0, Volts);
  // private static final MutVoltage outVolts = new MutVoltage(10, 0, Volts);
  public ColorSensorV3 colorSensor;
    
    public shooter() {
      backConfig.idleMode(IdleMode.kBrake);
      frontConfig.idleMode(IdleMode.kBrake);
      frontConfig.inverted(true);
      m_back.configure(backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_front.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      try{
        colorSensor= new ColorSensorV3(Port.kOnboard);
      }
      catch(Exception e){
        colorSensor = null;
    }
  }
  
  @Override
  public void periodic() {
    SmartDashboard.getNumber("Current Value m_back", m_back.getOutputCurrent());
    SmartDashboard.getNumber("Current Lowpassed Value m_back", lowpassFilter.calculate(m_back.getOutputCurrent()));
  }
  
  // -------- Movement methods ----------
  public void stopMotors() {
    m_back.stopMotor();
    m_front.stopMotor();
  }

  public void rollIntake() {
    // m_back.setVoltage(intakeVolts);
    m_back.set(0.65);
  }

  public void slowIntake(){
    m_back.set(0.2);
  }

  public void out() {
    // m_back.setVoltage(outVolts);
    // m_front.setVoltage(outVolts);
    m_back.set(0.5);
    m_front.set(0.5);
  }

  public void manualDrive(double frontPercent, double backPercent) {
    m_back.set(backPercent);
    m_front.set(frontPercent);
  }

  // --------- Other Methods ----------

  public boolean checkCoralFront() {
    return coralIsFront;
  }

  private boolean checkCoralGrabbing(){
    return lowpassFilter.calculate(m_back.getOutputCurrent()) > kCurrentThreshold;
  }

  // -------- Getter Methods-----------
  public ColorSensorV3 getSensor(){
    return colorSensor;
  }

  // -------- Lambda Commands ---------

  public Command manualShooterCommand(XboxController controller){
    return Commands.run(()->this.manualDrive(
      controller.getRawAxis(XboxController.Axis.kRightTrigger.value), 
      controller.getRawAxis(XboxController.Axis.kLeftTrigger.value)
    ), this);
  }

  public Command stopShooterCommand(){
    return Commands.runOnce(this::stopMotors);
  }

  public Command rollIntakeCommand(){
    return Commands.runOnce(this::rollIntake);
  }

  public SequentialCommandGroup intakeTimeCommand(){
    return new SequentialCommandGroup(
      Commands.runOnce(this::rollIntake),
      new WaitCommand(0.2),
      Commands.runOnce(this::stopMotors)
    );
  }

  public Command shootCommand(){
    return Commands.runOnce(this::out);
  }
}
