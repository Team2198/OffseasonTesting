// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticHub;
public class GearBox extends SubsystemBase {
  /** Creates a new GearBox. */
  DoubleSolenoid piston;
  Compressor compressor;
  PneumaticHub pMatics;
  DoubleSolenoid shooter;
  DoubleSolenoid cubeLocker;
  double counter = 1;
  public GearBox() {
    //compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    pMatics = new PneumaticHub(6);
    shooter = pMatics.makeDoubleSolenoid(14, 15);
    //piston = new DoubleSolenoid(PneumaticsModuleType.REVPH,7,6);
  piston = pMatics.makeDoubleSolenoid(5, 6); 
  cubeLocker = pMatics.makeDoubleSolenoid(0, 1);
    //compressor.enableDigital();
  }
  
 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("counter500", counter);
    counter+=1;
    // This method will be called once per scheduler run
  }

  

  

 

  public void shootCube(){
    shooter.set(DoubleSolenoid.Value.kReverse);
  }

  public void retractShooter(){
    shooter.set(DoubleSolenoid.Value.kForward);
  }
  

  public void lockCube(){
    cubeLocker.set(DoubleSolenoid.Value.kReverse);
  }

  public void releaseLock(){
    cubeLocker.set(DoubleSolenoid.Value.kForward);
  }

  





}
