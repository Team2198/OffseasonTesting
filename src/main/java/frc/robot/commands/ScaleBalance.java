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
import edu.wpi.first.wpilibj.Timer;
public class ScaleBalance extends CommandBase {
  /** Creates a new ScaleBalance. */
  DriveSubsystem m_robotDrive;
  PIDController pidController;
  DoublePublisher pitchPub;
  DoublePublisher setpointPub;
  
  //Logger logger;
  double lastTimestamp;
  public ScaleBalance(DriveSubsystem drive) {
    //logger = new Logger();
    m_robotDrive = drive;
    addRequirements(m_robotDrive);
    pidController = new PIDController(0.008, 0, 0);
    lastTimestamp = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //pitchPub = table.getDoubleTopic
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get the up and down angle (pitch) of the robot
    /* double angle = m_robotDrive.getPitch();
    //declaring a variable for the max voltage so too much voltage is not 
    //applied and the robot slides off
    double maxVoltage = 0.4; 
    
    double minVoltage = 0.24;
    //calculate the voltage that is needed to go forward or back to balance perfectly on the scale
    double voltage = -1 * MathUtil.clamp(pidController.calculate(angle, 0), minVoltage, maxVoltage);
    
    double setPointTolerance = 1;
    pidController.setTolerance(setPointTolerance); 
    
    
    //make the calculated voltage less if it is over the max voltage
    
    //apply the caculated voltage to the drive train 
    
    m_robotDrive.tankDriveVolts(voltage, voltage);
    double[] graphingData = {m_robotDrive.getPitch(), 0};
    SmartDashboard.putNumberArray("graphingdata", graphingData); */

    double angle = m_robotDrive.getPitch();
    //declaring a variable for the max voltage so too much voltage is not 
    //applied and the robot slides off
    
    //calculate the voltage that is needed to go forward or back to balance perfectly on the scale
    double voltage = pidController.calculate(angle, 0);
    SmartDashboard.putNumber("angle to balance", angle);
    SmartDashboard.putNumber("angle setpoint", 0);
    SmartDashboard.putNumber("voltage applied", voltage);
    double setPointTolerance = 2;
    pidController.setTolerance(setPointTolerance,2); 

    /* logger.log(Timer.getFPGATimestamp());
    logger.log(0);
    logger.log(angle);
    logger.log(pidController.getPositionError()); */
    //logger.log((Math.abs(error) < iZone) ? 1 : 0);
    //logger.log(errorSum);
    //logger.log(errorRate);

    //logger.log(kP * error);
    //logger.log(kI * errorSum);
    //logger.log(kD * errorRate);
   // logger.log(voltage);
    //logger.log("\n");
    
    
    //make the calculated voltage less if it is over the max voltage
    SmartDashboard.putNumber("Voltage balance", voltage);
    //apply the caculated voltage to the drive train 
    m_robotDrive.tankDriveVolts(-voltage, -voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //ends adjusting the robot on the scale as long as it is 
    //inside of a specific angle threshold where you are balanced
    if (pidController.atSetpoint()){
      return true;
    }
    
    return false;
  }
}
