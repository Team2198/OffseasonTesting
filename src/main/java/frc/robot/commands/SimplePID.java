// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SimplePID extends CommandBase {
  /** Creates a new SimplePID. */
  double initialPosition;
  PIDController pidController;
  DriveSubsystem drive;
  double turningAmount;
  double turningGoal;

  public SimplePID(DriveSubsystem drive) {
    this.drive = drive;
    
    pidController = new PIDController(0.029, 0, 0.005);
    addRequirements(this.drive);
    SmartDashboard.putData("pidcontroller",pidController);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setTolerance(0);

    initialPosition = drive.getYaw();
    turningAmount = 90;
    turningGoal = initialPosition + turningAmount; 
    if(turningGoal > 180.0){
      turningGoal = turningGoal - 360; 
    }
    
    
    else if(turningGoal<-180){
      turningGoal = turningGoal + 360;
    }
    SmartDashboard.putNumber("goal angle", turningGoal);
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double voltage = pidController.calculate(drive.getYaw(), initialPosition + turningAmount);
    if (voltage<0){
      voltage = voltage - 0.19;
    }
    else if (voltage>0){
      voltage = voltage + 0.19;
    }
    else{
      voltage = 0;
    }
    //voltage = voltage + 0.19*Math.signum(voltage);
   // SmartDashboard.putNumber("goal angle", (pidController.getSetpoint()));
    SmartDashboard.putNumber("current angle",drive.getYaw());
    SmartDashboard.putNumber("voltage",voltage);
    drive.arcadeDrive(0,voltage);
    
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
