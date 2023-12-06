// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SimpleFeedforward extends CommandBase {
  /** Creates a new SimplePID. */
  double initialPosition;
  PIDController pidController;
  DriveSubsystem drive;
  double turningAmount;
  double turningGoal;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1093, 0.10643);
  public SimpleFeedforward(DriveSubsystem drive) {
    this.drive = drive;
    
    pidController = new PIDController(0.0066, 0, 0);
    addRequirements(this.drive);
    SmartDashboard.putData("pidcontroller",pidController);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setTolerance(0);

    
    SmartDashboard.putNumber("goal angle", turningGoal);
    
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currVelocity = drive.getVelocity();
    double voltage = feedforward.calculate(35);
    voltage = voltage + pidController.calculate(drive.getVelocity(), 35);
    
    //voltage = voltage + 0.19*Math.signum(voltage);
    SmartDashboard.putNumber("goal velocty", 35);

    SmartDashboard.putNumber("velocity",currVelocity);
    SmartDashboard.putNumber("voltage",voltage);
    drive.arcadeDrive(voltage,0);

    
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
