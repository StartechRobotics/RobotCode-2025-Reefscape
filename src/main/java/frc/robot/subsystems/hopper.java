// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hopper extends SubsystemBase {

  public hopper() {
    close();
  }

  private final void open(){
  }

  private final void close(){
  }

  // Commands -----
  public Command openCommand(){
    return Commands.run(this::open, this);
  }

  public Command closeCommand(){
    return Commands.run(this::close, this);
  }

  @Override
  public void periodic(){
  }
}
