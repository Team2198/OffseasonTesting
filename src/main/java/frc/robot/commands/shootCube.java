// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.GearBox;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class shootCube extends CommandBase {
  /** Creates a new shootCube. */
  GearBox gbox;
  boolean shoot;
  public shootCube(GearBox gearBox, boolean shootInput) {
    gbox = gearBox;
    shoot = shootInput;
    addRequirements(gbox);

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
    if (shoot){
      gbox.shootCube();
    }
    else{
      gbox.retractShooter();
    }
    
    return true;
  }
}
