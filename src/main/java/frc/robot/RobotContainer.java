package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Controllers
  public XboxController drive_controller = new XboxController(Constants.ID_DRIVER_CHASSIS);
  public XboxController mech_controller = new XboxController(Constants.ID_DRIVER_MECH);
  public XboxController test_controller = new XboxController(Constants.ID_TEST_CONTROLLER);
  public PS4Controller ps4Controller = new PS4Controller(1);

  // Subsystems Instances
  public chassis m_chassis = new chassis(drive_controller);
  public shooter m_shooter = new shooter();
  public elevator m_elevator = new elevator();

  // public ColorSensorV3 shooterSensor = m_shooter.getSensor();

  // SysID Tests Triggers
  public Trigger dynamfwdTrigger = new JoystickButton(test_controller, XboxController.Button.kY.value);
  public Trigger quasifwdTrigger = new JoystickButton(test_controller, XboxController.Button.kA.value);
  public Trigger dynambwdTrigger = new JoystickButton(test_controller, XboxController.Button.kB.value);
  public Trigger quasibwdTrigger = new JoystickButton(test_controller, XboxController.Button.kLeftStick.value);
  public Trigger stopTestTrigger = new JoystickButton(test_controller, XboxController.Button.kX.value);

  // Chassis Triggers
  public Trigger stopChassisTrigger = new JoystickButton(drive_controller, XboxController.Button.kX.value);

  // Shooter Triggers
  public Trigger intakeTrigger = new JoystickButton(mech_controller, XboxController.Button.kA.value);
  public Trigger shooterStopTrigger = new JoystickButton(mech_controller, XboxController.Button.kX.value);
  public Trigger shootTrigger = new JoystickButton(mech_controller, XboxController.Button.kY.value);
  public Trigger intakeSequenceTrigger = new JoystickButton(mech_controller, XboxController.Button.kB.value);
  // public Trigger coralInTrigger = new Trigger(()->{return shooterSensor.getProximity() > Constants.kShooterSensorThreshold;});

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    defaultCommands();

    autoChooser = AutoBuilder.buildAutoChooser("auto 1");
    autoChooser.addOption("test auto", new PathPlannerAuto("test auto"));
    autoChooser.addOption("auto 0", new PathPlannerAuto("auto 0"));
    autoChooser.addOption("auto 0", new PathPlannerAuto("auto 1"));
    SmartDashboard.putData("AutoChooser",autoChooser);

    SmartDashboard.putData("Reset Gyro", m_chassis.resetGyroCommand(m_chassis));
    SmartDashboard.putData("Reset Odometry", m_chassis.resetOdometryCommand(m_chassis));
  }

  // -------- Methods ----------

  private void defaultCommands() {
    m_shooter.setDefaultCommand(m_shooter.manualShooterCommand(mech_controller, m_shooter));
    m_elevator.setDefaultCommand(m_elevator.driveCommand(m_elevator, mech_controller));
  }
  
  private void configureBindings() {
    stopChassisTrigger.onTrue(new SequentialCommandGroup(
      m_chassis.stopCommand(m_chassis), 
      m_chassis.clearFaultsCommand(m_chassis)
    ));
    intakeTrigger.onTrue(m_shooter.intakeCommand(m_shooter));
    shootTrigger.onTrue(m_shooter.shootCommand(m_shooter));
    shooterStopTrigger.onTrue(m_shooter.stopShooterCommand(m_shooter));
    intakeSequenceTrigger.onTrue(m_shooter.intakeTimeCommand(m_shooter));
    // coralInTrigger.onTrue(m_shooter.stopShooterCommand(m_shooter));
  }

  public void RobotCharacterizations(){
    // SysID TEST COMMAND BINDINGS - TEMP DISABLED
    dynamfwdTrigger.onTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kForward, m_elevator));
    quasifwdTrigger.onTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward, m_elevator));
    quasibwdTrigger.onTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse, m_elevator));
    dynambwdTrigger.onTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse, m_elevator));
    stopTestTrigger.onTrue(m_elevator.dryStopCommand(m_elevator));
  }

  public void StatesMachine(){
    // Add states machine here
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // Compound Commands  --------
  public Command shootAndDropSequence(){
    return new SequentialCommandGroup(
      m_shooter.shootCommand(m_shooter),
      new WaitCommand(1).andThen(m_shooter.stopShooterCommand(m_shooter)),
      m_elevator.driveToTargetCommand(Constants.kElevatorBottomPosition, m_elevator)
    );
  }
}

/*
 * CODE PENDING TASKS
 * - Do robot characterization tests for subsystem elevator
 * - Set the colorsensor to the shooter subsystem
 * - Add the color sensor to the shuffleboard for coral identification
 * - Add subsystems for passive mechanisms
 * - Add climber subsystem
 * - Add subsystem for vision processing
 * - Test PathPlannerAuto
 * - Configure voltages and thresholds for the subsystems: grabber, shooter, elevator
 * 
 */
