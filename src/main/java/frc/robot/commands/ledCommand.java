// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.Constants;

import edu.wpi.first.networktables.DoublePublisher;
public class ledCommand extends CommandBase {
  /** Creates a new ledCommand. */
  DriveSubsystem m_drive;
  int color;
  public ledCommand(DriveSubsystem drive, int color) {
    m_drive = drive;
    this.color = color;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (color==0){
      m_drive.setBlue();
    }
    else if (color == 1){
      m_drive.setRed();
    }
    else{
      m_drive.setPink();
    }
    
    return true;
  }
}
