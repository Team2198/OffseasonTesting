package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.DoublePublisher;
/** An example command that uses an example subsystem. */
public class  DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  DoubleSupplier xVoltage;
  DoubleSupplier yVoltage;
  
  double directionCoefficient;
  DoublePublisher pitchPub;
  DoublePublisher setpointPub;
  int count = 0;
  //private static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
  public DriveCommand(DriveSubsystem drive, DoubleSupplier y, DoubleSupplier x) {
    m_drive = drive;
    
    xVoltage = x;
    
    yVoltage = y;
    directionCoefficient = -1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //sets the drive voltages
    //m_drive.tankDriveVolts(y, y);
    // pitchPub = table.getDoubleTopic("pitch").publish();
    //setpointPub = table.getDoubleTopic("setpoint").publish();;
    SmartDashboard.putBoolean("inside command", true);
    SmartDashboard.putBoolean("inside command two", true);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("The drive Direction 2", "hello");
    //sets the drive voltages
   /*  if (directionButtonState.getAsBoolean()){
      directionCoefficient*=-1;
    } */
    /* SmartDashboard.putNumber("yaw", m_drive.getYaw());
    SmartDashboard.putNumber("pitch charge stATION", m_drive.getPitch());
    SmartDashboard.putNumber("left encoder 2", m_drive.leftEncoderReading()*1/16.36);//1/15 low speed gear ratio
    SmartDashboard.putNumber("x", yVoltage.getAsDouble()); */
    m_drive.arcadeDrive(yVoltage.getAsDouble(), xVoltage.getAsDouble());
    //m_drive.arcadeDrive(yVoltage.getAsDouble(), 0.19);
    count+=1;
    SmartDashboard.putNumber("LimelightX" , m_drive.getValueX());
    //double[] graphingData = {m_drive.getPitch(), 0};
    
    //pitchPub.set(m_drive.getPitch());
    //setpointPub.set(0);
    //NetworkTableEntry ta = table.put("pitch", m_drive.getPitch());
    //SmartDashboard.putBoolean("abutton2", directionCoefficient.getAsBoolean());
    //SmartDashboard.putNumber("height", Math.tan(Math.toRadians(m_drive.getValueX()))*59.5);
  }



  // Returns true when the command should end.
 
}
