// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SimplePID extends CommandBase {
  /** Creates a new SimplePID. */
  double initialPosition;
  PIDController pidController;
  DriveSubsystem drive;
  public SimplePID(DriveSubsystem drive) {
    this.drive = drive;
    pidController = new PIDController(1, 0, 0);
    addRequirements(this.drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPosition = drive.getAverageEncoderDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double voltage = pidController.calculate(drive.getAverageEncoderDistance(), initialPosition+120);
    drive.tankDriveVolts(voltage,voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
