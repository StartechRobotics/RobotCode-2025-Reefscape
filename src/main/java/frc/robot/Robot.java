package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final String defaultDriveController = "XboxDrive";
  private String controller_type_selected;
  private final SendableChooser<String> m_driveControllerChooser = new SendableChooser<>();

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_driveControllerChooser.setDefaultOption("Xbox controller", defaultDriveController);
  }

  @Override
  public void robotInit(){
    SmartDashboard.putData(m_driveControllerChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    controller_type_selected = m_driveControllerChooser.getSelected();
    switch (controller_type_selected) {
      default:
        m_robotContainer.m_chassis.setDefaultCommand(m_robotContainer.m_chassis.driveCommand(m_robotContainer.drive_controller));
      break;
    }
      
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
