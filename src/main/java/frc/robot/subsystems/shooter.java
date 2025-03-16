package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Volts;

// import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class shooter extends SubsystemBase {
  public final SparkMax m_back = new SparkMax(Constants.ID_SHOOTER_BACK, MotorType.kBrushless);
  public final SparkMax m_front = new SparkMax(Constants.ID_SHOOTER_FRONT, MotorType.kBrushless);
  public SparkMaxConfig backConfig = new SparkMaxConfig();
  public SparkMaxConfig frontConfig = new SparkMaxConfig();

  // private static final MutVoltage intakeVolts = new MutVoltage(8.5, 0, Volts);
  // private static final MutVoltage outVolts = new MutVoltage(10, 0, Volts);
  // public final ColorSensorV3 colorSensor = new ColorSensorV3(Port.kOnboard);

  public shooter() {
    backConfig.idleMode(IdleMode.kBrake);
    frontConfig.idleMode(IdleMode.kBrake);
    frontConfig.inverted(true);

    m_back.configure(backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_front.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  // -------- Movement methods ----------
  public void stopMotors(){
    m_back.stopMotor();
    m_front.stopMotor();
  }

  public void rollIntake(){
    // m_back.setVoltage(intakeVolts);
    m_back.set(0.65);
  }

  public void out(){
    // m_back.setVoltage(outVolts);
    // m_front.setVoltage(outVolts);
    m_back.set(0.5);
    m_front.set(0.5);
  }

  public void manualDrive(double frontPercent, double backPercent){
    m_back.set(backPercent);
    m_front.set(frontPercent);
  }

  // -------- Getter Methods-----------
  // public ColorSensorV3 getSensor(){
  //   return colorSensor;
  // }
  // -------- Lambda Commands ---------

  public Command manualShooterCommand(XboxController controller){
    return Commands.run(()->this.manualDrive(
      controller.getRawAxis(XboxController.Axis.kRightTrigger.value), 
      controller.getRawAxis(XboxController.Axis.kLeftTrigger.value)
    ), this);
  }

  public Command stopShooterCommand(){
    return Commands.runOnce(() -> this.stopMotors());
  }

  public Command intakeCommand(){
    return Commands.runOnce(()-> this.rollIntake());
  }

  public SequentialCommandGroup intakeTimeCommand(){
    return new SequentialCommandGroup(
      Commands.runOnce(()->this.rollIntake()),
      new WaitCommand(0.2),
      Commands.runOnce(()->this.stopMotors())
    );
  }

  public Command shootCommand(){
    return Commands.runOnce(()->this.out());
  }
}
