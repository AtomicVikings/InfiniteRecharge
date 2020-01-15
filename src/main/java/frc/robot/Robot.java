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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//Motor Controllers
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

//Drive
import edu.wpi.first.wpilibj.drive.DifferentialDrive; 
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Robot extends TimedRobot {
  //I2C fpr Color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  
  //Drive 
  private DifferentialDrive drive;
  private TalonFX fRight, fLeft, rRight, rLeft; 
  //Mechanisms
  private CANSparkMax leftShootyBoi, rightShootyBoi;
  
  @Override
  public void robotInit() {
    //Mech
    leftShootyBoi = new CANSparkMax(5, MotorType.kBrushless);
    rightShootyBoi = new CANSparkMax(6, MotorType.kBrushless);
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
}
