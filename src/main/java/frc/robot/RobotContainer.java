package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Controllers
  public XboxController drive_controller = new XboxController(Constants.ID_DRIVER_CHASSIS);
  public XboxController mech_controller = new XboxController(Constants.ID_DRIVER_MECH);
  public boolean mech_controllerLeftAxToBool (){
    return Math.abs(drive_controller.getRawAxis(XboxController.Axis.kLeftY.value))>0.5;
  }
  public XboxController test_controller = new XboxController(Constants.ID_TEST_CONTROLLER);

  // Subsystems Instances
  public chassis m_chassis = new chassis();
  public shooter m_shooter = new shooter();
  public elevator m_elevator = new elevator();
  // public hopper m_hopper = new hopper();
  // public grabber m_grabber = new grabber();

  // // SysID Tests Triggers
  // public Trigger dynamfwdTrigger = new JoystickButton(test_controller, XboxController.Button.kY.value);
  // public Trigger quasifwdTrigger = new JoystickButton(test_controller, XboxController.Button.kA.value);
  // public Trigger dynambwdTrigger = new JoystickButton(test_controller, XboxController.Button.kB.value);
  // public Trigger quasibwdTrigger = new JoystickButton(test_controller, XboxController.Button.kLeftStick.value);
  // public Trigger stopTestTrigger = new JoystickButton(test_controller, XboxController.Button.kX.value);

  // Chassis Triggers
  public Trigger stopChassisTrigger = new JoystickButton(drive_controller, XboxController.Button.kX.value);

  // Shooter Triggers
  public BooleanSupplier axisGreater = this::mech_controllerLeftAxToBool;
  public Trigger intakeTrigger = new Trigger(axisGreater);
  public Trigger shooterStopTrigger = new JoystickButton(mech_controller, XboxController.Button.kX.value);
  public Trigger shootTrigger = new JoystickButton(mech_controller, XboxController.Button.kY.value);
  // public Trigger intakeSequenceTrigger = new JoystickButton(mech_controller, XboxController.Button.kB.value);
  public Trigger hasCoralFront = new Trigger(m_shooter::checkCoralFront);
  // Elevator Triggers
  public Trigger targetTrigger = new JoystickButton(mech_controller, XboxController.Button.kA.value);

  // Hopper Triggers
  public Trigger openHoppTrigger = new JoystickButton(test_controller, XboxController.Button.kA.value);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    defaultCommands();

    autoChooser = AutoBuilder.buildAutoChooser("auto 1");
    autoChooser.setDefaultOption("test auto (Default Momentan)", new PathPlannerAuto("test auto"));
    autoChooser.addOption("auto 0", new PathPlannerAuto("auto 0"));
    autoChooser.addOption("auto 1", new PathPlannerAuto("auto 1"));
    SmartDashboard.putData("AutoChooser",autoChooser);

    SmartDashboard.putData("Reset Gyro", m_chassis.resetGyroCommand());
    SmartDashboard.putData("Reset Odometry", m_chassis.resetOdometryCommand());
  }

  // -------- Methods ----------

  private void defaultCommands() {
    m_shooter.setDefaultCommand(m_shooter.manualShooterCommand(mech_controller));
    m_elevator.setDefaultCommand(m_elevator.driveCommand(mech_controller));
    // m_hopper.setDefaultCommand(m_hopper.closeCommand());
  }
  
  private void configureBindings() {
    stopChassisTrigger.onTrue(new SequentialCommandGroup(
      m_chassis.stopCommand(), 
      m_chassis.clearFaultsCommand()
    ));
    intakeTrigger.onTrue(m_shooter.rollIntakeCommand()).onFalse(m_shooter.stopShooterCommand());
    shootTrigger.onTrue(m_shooter.shootCommand());
    shooterStopTrigger.onTrue(m_shooter.stopShooterCommand());
    // intakeSequenceTrigger.onTrue(m_shooter.intakeTimeCommand());
    // openHoppTrigger.onTrue(m_hopper.openCommand()).onFalse(m_hopper.closeCommand()); 
  }

  public void RobotCharacterizations(){
    // SysID TEST COMMAND BINDINGS - TEMP DISABLED
    // dynamfwdTrigger.onTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // quasifwdTrigger.onTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // quasibwdTrigger.onTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // dynambwdTrigger.onTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // stopTestTrigger.onTrue(m_elevator.dryStopCommand());
  }

  public void StatesMachine(){
    targetTrigger.and(hasCoralFront).onTrue(m_elevator.driveToTargetCommand(0));
  }
  
  public Command getAutonomousCommand() {
    Command autoCommand = autoChooser.getSelected();
    return autoCommand != null ? autoCommand : new WaitCommand(0); // Evita NullPointerException
  }

  // Compound Commands  --------
  public Command shootAndDropSequence(){
    return new SequentialCommandGroup(
      m_shooter.shootCommand(),
      new WaitCommand(1).andThen(m_shooter.stopShooterCommand()),
      m_elevator.driveToTargetCommand(Constants.kElevatorBottomPosition)
    );
  }
}

/*
 * CODE PENDING TASKS
 * - Do robot characterization tests for subsystem elevator
 * - Set the colorsensor to the shooter subsystem
 * - Add the color sensor to the shuffleboard for coral identification
 * - Add climber subsystem
 * - Add subsystem for vision processing
 * - Test PathPlannerAuto
 * - Configure voltages and thresholds for the subsystems: grabber, shooter, elevator
 * 
 */
