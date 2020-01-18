//Neccesary Packages
package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

//Smart Dashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CAN;

//Color Stuffs
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
//Motor Controllers
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//Drive
import edu.wpi.first.wpilibj.drive.DifferentialDrive; 
//NetworkTables
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Robot extends TimedRobot {
  
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
  private TalonFX           topRightDriveBoi, topLeftDriveBoi, bottomRightDriveBoi, bottomLeftDriveBoi;

  //Mechanisms
  private CANSparkMax intakeyBoi, rolleyBoi, turretBoi, leftClimbyBoi, rightClimbyBoi;
  private VictorSPX   conveyorBoi;
  private TalonFX     leftShootyBoi, rightShootyBoi;
  
  @Override
  public void robotInit() {
    //Mech
    leftShootyBoi = new TalonFX(5);
    rightShootyBoi = new TalonFX(6);
  }

  @Override
  public void robotPeriodic() {
    final Color detectedColor = colorSensor.getColor();
    final double IR = colorSensor.getIR();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    final int proximity = colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);
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

  public void NetTableVals() {
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
}
