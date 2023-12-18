package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveSubsystem extends SubsystemBase {


  // The motors on the left side of the drive.
  

  private final CANSparkMax leftMotorOne = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax leftMotorTwo = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax rightMotorOne = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightMotorTwo = new CANSparkMax(3, MotorType.kBrushless);
  
  private final MotorControllerGroup m_leftMotors =
  new MotorControllerGroup(
      leftMotorOne,
      leftMotorTwo);

  private final MotorControllerGroup m_rightMotors =
  new MotorControllerGroup(
      rightMotorOne,
      rightMotorTwo);

  private final Spark led; 

  //declaring gyro

  // The motors on the right side of the drive.
  

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final RelativeEncoder m_leftEncoder = leftMotorOne.getEncoder();
  private final RelativeEncoder m_rightEncoder = rightMotorOne.getEncoder();
      

  // The right-side drive encoder
  

      

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();
  //kinematics for the odometry for the robot
  
  // Odometry class for tracking robot pose need to add std deviations 
  //of gyro and encoders to get more accuratepose
  
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotors.setInverted(true);
    led = new Spark(4);
    
    
    
   
  }

  @Override
  public void periodic() {   
    //getValueX();
   // getValueY(); 
    SmartDashboard.putNumber("Coounts PR",m_leftEncoder.getCountsPerRevolution());  
    
    
    // Update the odometry in the periodic block to keep track of robot pose
    
  }

  public void setBlue(){
    led.set(0.85);
  }

  public void setRed(){
    led.set(0.61);
  }

  public void setPink(){
    led.set(0.57);
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */



public void setPipeline(int pipeline){
  table.getEntry("pipeline").setNumber(pipeline);
  
}

public double getVelocity(){
  return (m_leftEncoder.getVelocity()+m_rightEncoder.getVelocity())/2*1/16.9;
}






   //getting and placing tx values on smart dashboard 
public double getValueX(){ 
  //tx values for limelight 

NetworkTableEntry tx = table.getEntry("tx");

//Reading x values from limelight
double x = tx.getDouble(0.0); 

//Updating smart Dashboard
SmartDashboard.putNumber("LimelightX" , x);

return x;
}



//getting and placing ty values on smart dashboard 
public double getValueY(){ 
  //  ty values for limelight 
  
  NetworkTableEntry ty = table.getEntry("ty");

  //Reading ty values from limelight
  double y = ty.getDouble(0.0); 

  //Updating smart Dashboard
  SmartDashboard.putNumber("LimelightY", y);
  return y;
}

public double distanceFromTarget(){
  //calculating the height of the triangle
  double height = 46 - 0;
  //tan(angle) = height/distance
  //distance = height/tan(angle)
  double distance = height/Math.tan(getValueX());
  return distance - 39.75;
}


//getting and placing ta values on smart dashboard 
public double getValueA(){ 
  
  NetworkTableEntry ta = table.getEntry("ta");

//Reading ty values from limelight
  double a = ta.getDouble(0.0); 

//Updating smart Dashboard
  SmartDashboard.putNumber("LimelightA", a);
  return a;
  }
  
  
  




  /**
   * Returns the current wheel speeds of the robot.
   *
   * return The current wheel speeds.
   */
  //get current wheel speeds
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public double getSpeedRight(){
    return m_rightEncoder.getVelocity();
  }
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  


  

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  //method for arcade drive takes the 
  public void arcadeDrive(double x, double y) {
    m_drive.arcadeDrive(x, y);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  //method for tank drive so pid voltages and trajectory tracking voltages can be applied
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts*12);
    m_rightMotors.setVoltage(rightVolts*12);

  }

  /** Resets the drive encoders to currently read a position of 0. */
  

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    double rotations = (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0 * 1/16.9; 
    SmartDashboard.putNumber("Revolutions", rotations);
    SmartDashboard.putNumber("distance", rotations*6*Math.PI);
    return rotations*6*Math.PI;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }


  //return kinematics object to be used in the auto commands
 

  public double getDistanceLeft(){
    return m_leftEncoder.getPosition();

  }

  //get the yaw (side to side) angle of the drive 
  public double getYaw(){
    return m_gyro.getYaw();
  }

  //get the pitch (up and down) angle of the drive 
  public double getPitch(){
    return m_gyro.getPitch();
  }
}