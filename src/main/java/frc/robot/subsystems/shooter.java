package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.nio.file.FileSystemAlreadyExistsException;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
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
  private final RelativeEncoder frontEncoder = m_front.getEncoder();
  private SparkMaxConfig backConfig = new SparkMaxConfig();
  private SparkMaxConfig frontConfig = new SparkMaxConfig();
  private LinearFilter lowpassFilter = LinearFilter.singlePoleIIR(0.02, 0.02);
  public boolean grabbingCoral = false;
  public boolean coralInBetween;
  public boolean coralIsFront;
  
  // private static final MutVoltage intakeVolts = new MutVoltage(8.5, 0, Volts);
  // private static final MutVoltage outVolts = new MutVoltage(10, 0, Volts);
  public ColorSensorV3 colorSensor;
    
    public shooter() {
      coralIsFront  = true;
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
    checkCoralFront();
    
    dasboardLabels();
  }

  public void dasboardLabels(){
    SmartDashboard.putNumber("Current Value m_back", m_back.getOutputCurrent());
    SmartDashboard.putNumber("Current Lowpassed Value m_back", lowpassFilter.calculate(m_back.getOutputCurrent()));
    SmartDashboard.putBoolean("Has Coral", coralIsFront);
    SmartDashboard.putBoolean("CoralInBetween", coralInBetween);
    SmartDashboard.putNumber("Front Encoder", frontEncoder.getPosition()*18/24);
    SmartDashboard.putBoolean("Grabbing", grabbingCoral);
  }
  
  // -------- Movement methods ----------
  public void stopMotors() {
    m_back.stopMotor();
    m_front.stopMotor();
  }

  public void rollIntake() {
    // m_back.setVoltage(intakeVolts);
    m_back.set(0.2);
  }

  public void slowIntake(){
    m_back.set(0.2);
  }

  public void out() {
    // m_back.setVoltage(outVolts);
    // m_front.setVoltage(outVolts);
    m_back.set(0.2);
    m_front.set(0.2);
  }

  public void takeBack(){
    m_back.set(-0.15);
    m_front.set(-0.15);
  }

  public void manualDrive(double frontPercent, double backPercent) {
    m_back.set(backPercent);
    m_front.set(frontPercent);
  }

  // --------- Control Methods ----------

  public boolean checkCoralInBetween() {
    coralInBetween = colorSensor.getProximity() < kProximityThreshold;
    return coralInBetween;
  }

  private boolean checkCoralGrabbing(){
    return frontEncoder.getPosition()*18/24 > 0;
  }
  
  private boolean checkCoralFront(){
    if(!coralIsFront && checkCoralGrabbing()&& !checkCoralInBetween()){
      coralIsFront = true;
      if(frontEncoder.getVelocity()==0){
      frontEncoder.setPosition(0);
      }
    }
    return coralIsFront;
  }

  public boolean hasCoral(){
    return coralIsFront;
  }

  private void resetCoralStatus(){
    coralIsFront = false;
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

  public Command takeBackCommand(){
    return Commands.runOnce(this::takeBack);
  }

  public Command shootCommand(){
    return new SequentialCommandGroup(
      Commands.runOnce(this::out), 
      new WaitCommand(0.5), 
      Commands.runOnce(this::stopMotors), 
      Commands.runOnce(this::resetCoralStatus));
  }
}
