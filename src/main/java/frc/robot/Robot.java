//Neccesary Packages
package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

//Smart Dashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Color Stuffs
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

//Motor Controllers
import edu.wpi.first.wpilibj.CAN;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

//Drive
import edu.wpi.first.wpilibj.drive.DifferentialDrive; 

//NetworkTables
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Robot extends TimedRobot {
  //NAVX
  private AHRS ahrs;

  //NetworkTable
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx"); //horizontal, from -27 to 27
  NetworkTableEntry ty = table.getEntry("ty"); //vertical, from -20.5 to 20.5
  NetworkTableEntry ta = table.getEntry("ta"); //target area, 0% to 100% of image

  //I2C fpr Color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  
  //Drive 
  private DifferentialDrive drive;
  private WPI_TalonFX           rightFront, leftFront, rightRear, leftRear;
  private SpeedControllerGroup leftSide, rightSide;

  //Mechanisms
  private CANSparkMax intakeyBoi, rolleyBoi, turretBoi, leftClimbyBoi, rightClimbyBoi;
  private VictorSPX   conveyorBoi;
  private WPI_TalonFX     leftShootyBoi, rightShootyBoi;
  
  @Override
  public void robotInit() {
    //Mech
    leftShootyBoi = new WPI_TalonFX(5);
    rightShootyBoi = new WPI_TalonFX(6);

    //NAVX
    try {
      ahrs = new AHRS(I2C.Port.kMXP);
    } catch (RuntimeException ex) {
      SmartDashboard.putString("Error", "There was an error initing the navX: " + ex.getMessage());
    }
    
    //Drive
    rightFront = new WPI_TalonFX(7);
    leftFront = new WPI_TalonFX(8);
    rightRear = new WPI_TalonFX(9);
    leftRear = new WPI_TalonFX(10);
    leftSide = new SpeedControllerGroup(leftFront, leftRear);
    rightSide = new SpeedControllerGroup(rightFront, rightRear);

    DifferentialDrive drive = new DifferentialDrive(leftSide, rightSide);
  }

  @Override
  public void robotPeriodic() {
    //Color Sensor
      final Color detectedColor = colorSensor.getColor();
      final double IR = colorSensor.getIR();

      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("IR", IR);

      final int proximity = colorSensor.getProximity();

      SmartDashboard.putNumber("Proximity", proximity);
    // Limelight
      
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testPeriodic() {

  }

  void OperatorControl() {
    float left_command = 
  }
}
