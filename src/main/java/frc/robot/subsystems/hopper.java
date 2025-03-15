// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class hopper extends SubsystemBase {
  private final Solenoid m_hopper = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ID_HOPPER_SOLENOID);

  public hopper() {
    close();
  }

  private final void open(){
    m_hopper.set(true);
  }

  private final void close(){
    m_hopper.set(false);
  }

  // Commands -----
  public Command openCommand(){
    return Commands.run(()->open(), this);
  }

  public Command closeCommand(){
    return Commands.run(()->close(), this);
  }

  @Override
  public void periodic(){
    SmartDashboard.putBoolean("Solenoid", m_hopper.get());
  }
}
