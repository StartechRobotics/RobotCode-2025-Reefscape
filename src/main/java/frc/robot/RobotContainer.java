package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  
  public boolean mech_controllerLeftAxToBoolGreater() {
    return mech_controller.getRawAxis(XboxController.Axis.kLeftY.value) > 0.5;
  }
  public boolean mech_controllerLeftAxToBoolLessThan() {
    return mech_controller.getRawAxis(XboxController.Axis.kLeftY.value) < -0.5;
  }
  public boolean mech_controllerRightAxToBool() {
    return Math.abs(mech_controller.getRawAxis(XboxController.Axis.kRightY.value)) > 0.5;
  }
  public boolean isL1() {
    return m_elevator.position == 1;
  }
  public double elevatorSafetyPercent(){
    return isL1() ? 1 : 0.2;
  }

  // Subsystems Instances
  public chassis m_chassis = new chassis();
  public shooter m_shooter = new shooter();
  public elevator m_elevator = new elevator();
  public grabber m_grabber = new grabber();

  // // SysID Tests Triggers
  public Trigger dynamfwdTrigger = new JoystickButton(test_controller,
  XboxController.Button.kY.value);
  public Trigger quasifwdTrigger = new JoystickButton(test_controller,
  XboxController.Button.kA.value);
  public Trigger dynambwdTrigger = new JoystickButton(test_controller,
  XboxController.Button.kB.value);
  public Trigger quasibwdTrigger = new JoystickButton(test_controller,
  XboxController.Button.kLeftStick.value);
  public Trigger stopTestTrigger = new JoystickButton(test_controller,
  XboxController.Button.kX.value);

  // Chassis Triggers
  public Trigger stopChassisTrigger = new JoystickButton(drive_controller, XboxController.Button.kX.value);

  // Shooter Triggers
  public Trigger intakeTrigger = new JoystickButton(mech_controller, XboxController.Button.kRightBumper.value);
  public Trigger shootTrigger = new JoystickButton(mech_controller, XboxController.Button.kLeftBumper.value);
  public Trigger isTrackFree = new Trigger(m_shooter::checkCoralInBetween);
  public Trigger hasCoral = new Trigger(m_shooter::hasCoral);

  // Oscar Triggers
  private final Trigger l1Trigger = new JoystickButton(mech_controller, XboxController.Button.kB.value);
  private final Trigger l2Trigger = new JoystickButton(mech_controller, XboxController.Button.kA.value);
  private final Trigger l3Trigger = new JoystickButton(mech_controller, XboxController.Button.kX.value);
  private final Trigger l4Trigger = new JoystickButton(mech_controller, XboxController.Button.kY.value);

  private final Trigger coralOutTrigger = new JoystickButton(mech_controller, XboxController.Button.kLeftBumper.value);
  private final Trigger coralInTrigger = new JoystickButton(mech_controller, XboxController.Button.kStart.value);
  private final Trigger shooterBackwardTrigger = new Trigger(this::mech_controllerRightAxToBool);
  private final Trigger shiftTrigger = new JoystickButton(mech_controller, XboxController.Button.kStart.value);

  // Elevator Triggers
  private final Trigger isL1Trigger = new Trigger(this::isL1);
  // Grabber Triggers
  public BooleanSupplier axisGreater = this::mech_controllerLeftAxToBoolGreater;
  public BooleanSupplier axisLessThan = this::mech_controllerLeftAxToBoolLessThan;
  public Trigger grabTrigger = new Trigger(axisGreater);
  public Trigger dropTrigger = new Trigger(axisLessThan);
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
    m_chassis.setDefaultCommand(m_chassis.driveCommand(drive_controller,elevatorSafetyPercent()));
    m_elevator.setDefaultCommand(m_elevator.driveCommand(mech_controller));
  }

  private void configureBindings() {
    // stopChassisTrigger.onTrue(new SequentialCommandGroup(
        // m_chassis.stopCommand(),
        // m_chassis.clearFaultsCommand()));
    

    // Oscar Triggers
    OscarTriggers(false);

    intakeTrigger.and(isL1Trigger).onTrue(m_shooter.rollIntakeCommand());
    // intakeSequenceTrigger.onTrue(m_shooter.intakeTimeCommand());
  }

  public void OscarTriggers(boolean isSimulation){
    l1Trigger.and(isTrackFree).onTrue(m_elevator.driveToTargetCommand(Constants.kL1Position));
    l2Trigger.and(isTrackFree).onTrue(m_elevator.driveToTargetCommand(Constants.kL2Position));
    l3Trigger.and(isTrackFree).onTrue(m_elevator.driveToTargetCommand(Constants.kL3Position));
    l4Trigger.and(isTrackFree).onTrue(m_elevator.driveToTargetCommand(Constants.kL4Position));

    shooterBackwardTrigger.onTrue(m_shooter.takeBackCommand()).onFalse(m_shooter.stopShooterCommand());

    intakeTrigger.onTrue(m_shooter.rollIntakeCommand().alongWith(intakeRumble()));
    shootTrigger
      .onTrue(
        shootRumble()
        .andThen(m_shooter.shootCommand())
        .andThen(noRumble())
      );
    shiftTrigger
      .onTrue(
        m_shooter.stopShooterCommand()
        .alongWith(noRumble())
      );

    grabTrigger.onTrue(m_grabber.grabCommand()).onFalse(m_grabber.stopCommand());
    dropTrigger.onTrue(m_grabber.dropCommand()).onFalse(m_grabber.stopCommand());
  } 

  public void RobotCharacterizations() {
    // SysID TEST COMMAND BINDINGS - TEMP DISABLED
    // dynamfwdTrigger.onTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // quasifwdTrigger.onTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // quasibwdTrigger.onTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // dynambwdTrigger.onTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // stopTestTrigger.onTrue(m_elevator.dryStopCommand());
  }

  public void StatesMachine() {
    hasCoral.onTrue(m_shooter.stopShooterCommand());
  }

  public Command getAutonomousCommand() {
    // Command autoCommand = autoChooser.getSelected();
    Command autoCommand = null;
    return autoCommand != null ? autoCommand : new WaitCommand(0); // Evita NullPointerException
  }

  // Custom Commands --------
  public Command intakeRumble(){
    return Commands.runOnce(()->mech_controller.setRumble(RumbleType.kLeftRumble, 0.7));
  }
  public Command shootRumble(){
    return Commands.runOnce(()->mech_controller.setRumble(RumbleType.kRightRumble, 0.7));
  }
  public Command noRumble(){
    return Commands.runOnce(()->mech_controller.setRumble(RumbleType.kBothRumble, 0));
  }

  public Command shootAndDropSequence() {
    return new SequentialCommandGroup(
        m_shooter.shootCommand(),
        new WaitCommand(1).andThen(m_shooter.stopShooterCommand()),
        m_elevator.driveToTargetCommand(Constants.kL1Position));
  }
}

/*
 * CODE PENDING TASKS
 * - Do robot characterization tests for subsystem elevator (Pit)
 * - Add climber subsystem (Discontinued)
 * - Add subsystem for vision processing (Discontinued)
 * - Test PathPlannerAuto
 * - Configure voltages and thresholds for the subsystems: grabber, shooter,
 * elevator
 * - Change to NavX for the gyro
 * 
 */
