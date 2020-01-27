//Neccesary Packages
package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//Smart Dashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.SerialPort;
import com.kauailabs.navx.frc.AHRS;

//Pneumatics
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

//Color Stuffs
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
//Motor Controllers
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//Drive
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;

//NetworkTables
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Robot extends TimedRobot {
  //AHRS ahrs;

  /*Things?
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String autoSelected;
  private final SendableChooser<String> chooser = new SendableChooser<>();*/
  
  //Pneumatics
  Compressor compressor;
  Solenoid solenoid;

  //NetworkTable
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx"); //horizontal, from -27 to 27
  NetworkTableEntry ty = table.getEntry("ty"); //vertical, from -20.5 to 20.5
  NetworkTableEntry ta = table.getEntry("ta"); //target area, 0% to 100% of image

  //I2C fpr Color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  
  private Timer timer = new Timer();
  //Drive 
  private DifferentialDrive drive;
  private SpeedControllerGroup leftDrive, rightDrive;
  private WPI_TalonFX          topRightDrive, topLeftDrive, bottomRightDrive, bottomLeftDrive;

  //Mechanisms
  private CANSparkMax intakey, rolley, leftClimby, rightClimby;
  private VictorSPX   conveyor;
  private WPI_TalonFX leftShooty, rightShooty, turret;

  //Controllers
  private Joystick logitechAlpha, logitechBeta;

  //Limelight stuffs
  private boolean LimelightHasTarget = false;
  private double LimelightDriveCommand = 0.0;
  private double LimelightSteerCommand = 0.0;

  //private AHRS ahrs;

  //Use for any initialization, is called whenever robot is started
  @Override
  public void robotInit() {
    //Drive
    topLeftDrive = new WPI_TalonFX(7);
    bottomRightDrive = new WPI_TalonFX(6);
    topRightDrive = new WPI_TalonFX(5);
    bottomLeftDrive = new WPI_TalonFX(8);

    leftDrive = new SpeedControllerGroup(topLeftDrive, bottomLeftDrive);
    rightDrive = new SpeedControllerGroup(topRightDrive, bottomRightDrive);

    drive = new DifferentialDrive(leftDrive, rightDrive);
    
    //Controller
    logitechAlpha = new Joystick(0);
    logitechBeta = new Joystick(1);

    //Mech
    leftShooty = new WPI_TalonFX(9);
    rightShooty = new WPI_TalonFX(10);
    turret = new WPI_TalonFX(15);


    solenoid = new Solenoid(11);
    compressor = new Compressor(12);
    //NavX 
    //ahrs = new AHRS(I2C.Port.kMXP);
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

    //Limelight
    //read values periodically
    final double x    = tx.getDouble(0.0);
    final double y    = ty.getDouble(0.0);
    final double area = ta.getDouble(0.0);
    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

  }

  //Called when robot enters autonomous
  @Override
  public void autonomousInit() {
    
  }

  //called periodically during autonomous
  @Override
  public void autonomousPeriodic() {
    while (timer.get() < 2.0) {
      drive.arcadeDrive(0.5, 0.0);
    }

    while (timer.get() < 5.0) {
      drive.arcadeDrive(0.5, 0.4);
    }

  }

  //called when robot enters tele-operated mode
  @Override
  public void teleopInit() {

  }

  //called periodically in tele-operated mode
  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(logitechAlpha.getRawAxis(1), logitechAlpha.getRawAxis(4));
    //Update_Limelight_Tracking();
    if (logitechAlpha.getRawButton(4) == true) {
      turret.set(1);
    } else if(logitechAlpha.getRawButton(5) == true) {
      turret.set(-1);
    }
  }

  @Override
  public void testPeriodic() {
  }
  /*
  public void Update_Limelight_Tracking() {
    final double STEER_K = 0.03;
    final double DRIVE_K = 0.26;
    final double DESIRED_TARGET_AREA = 13.0;
    final double MAX_DRIVE = 0.7;

    final double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    final double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    final double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    final double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if (tv < 1.0) {
      LimelightHasTarget = false;
      LimelightDriveCommand = 0.0;
      LimelightSteerCommand = 0.0;
    }

    LimelightHasTarget = true;

    // start with proportional steering
    double steer_cmd = tx * STEER_K;
    LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    LimelightDriveCommand = drive_cmd;

  }

  public void operatorControl() {
    while (isOperatorControl() && isEnabled()) {
      Timer.delay(0.020); // wait for one motor time period (50Hz) 

      boolean zero_yaw_pressed = logitechAlpha.getTrigger();
      if (zero_yaw_pressed) {
        ahrs.zeroYaw();
      }

      // Display Processed Acceleration Data (Linear Acceleration, Motion Detect) 
      SmartDashboard.putNumber(  "IMU_Accel_X",    ahrs.getWorldLinearAccelX());
      SmartDashboard.putNumber(  "IMU_Accel_Y",    ahrs.getWorldLinearAccelY());
      SmartDashboard.putBoolean( "IMU_IsMoving",   ahrs.isMoving());
      SmartDashboard.putBoolean( "IMU_IsRotating", ahrs.isRotating());

      SmartDashboard.putNumber(  "RawAccel_X",     ahrs.getRawAccelX());
      SmartDashboard.putNumber(  "RawAccel_Y",     ahrs.getRawAccelY());
      SmartDashboard.putNumber(  "RawAccel_Z",     ahrs.getRawAccelZ());
      
    }
  }
  */
}