// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogGyro;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private static final int PH_CAN_ID = 11;
  private static int forwardChannel = 3;
  private static int reverseChannel = 0;
  PneumaticHub m_ph = new PneumaticHub(PH_CAN_ID);
  DoubleSolenoid m_doubleSolenoid = m_ph.makeDoubleSolenoid(forwardChannel,reverseChannel);
  
  private final XboxController m_controller = new XboxController(0);
  private final Talon m_leftDrive0 = new Talon(0);
  private final Talon m_leftDrive1 = new Talon(1);
  private final Talon m_rightDrive2 = new Talon(2);
  private final Talon m_rightDrive3 = new Talon(3);
  //private final PWMSparkMax m_pulley = new PWMSparkMax(4);
  private final CANSparkMax m_pulley = new CANSparkMax(4,MotorType.kBrushed);

  private final CANSparkMax m_grip = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax m_flip = new CANSparkMax(5, MotorType.kBrushless);
  // private final PWMSparkMax m_grip = new PWMSparkMax(7);
  // private final PWMSparkMax m_flip = new PWMSparkMax(5);
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_leftDrive0,m_leftDrive1);
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightDrive2,m_rightDrive3);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);
  private final Timer m_timer = new Timer();
  double lastGripIn = 0;
  double lastGripOut = 0;
  double lastFlip = 0;
  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private boolean m_gripState = false;
  //--------------------------------------------------------------------------------------------------------
  //Camera Code
  //--------------------------------------------------------------------------------------------------------
  Thread m_visionThread;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //CameraServer.startAutomaticCapture();
  
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // Add buttons to set the double
    m_right.setInverted(true);
    m_left.setInverted(false);
    m_grip.setSmartCurrentLimit(50);
    m_flip.setSmartCurrentLimit(30);
    m_pulley.setSmartCurrentLimit(50);
    // m_pulley.setSafetyEnabled(false);
    // m_pulley.setInverted(true);
  
    SmartDashboard.setDefaultBoolean("Set Off", false);
    SmartDashboard.setDefaultBoolean("Set Forward", false);
    SmartDashboard.setDefaultBoolean("Set Reverse", false);
    SmartDashboard.setDefaultBoolean("Enable Compressor Digital", false);
    SmartDashboard.setDefaultBoolean("Disable Compressor", false);

    m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Rotated", 480, 640);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                double angle = 270;
                Point center = new Point(mat.width()/2,mat.height()/2);
                Mat rotationMatrix = Imgproc.getRotationMatrix2D(center, angle, 1);
                Mat rotatedImage = new Mat();
                Imgproc.warpAffine(mat, rotatedImage, rotationMatrix, mat.size());
      
                outputStream.putFrame(rotatedImage);
              }
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }
//------------------------------------------------------------------------------------------------
//** Robot periodic */
@Override
  public void robotPeriodic() {
    /**
     * Get digital pressure switch state and display on Shuffleboard
     */
    SmartDashboard.putBoolean("Digital Pressure Switch",
    m_ph.getPressureSwitch());
   
    /**
    * Get compressor running status and display on Shuffleboard.
    */
    SmartDashboard.putBoolean("Compressor Running", m_ph.getCompressor());

    // Enable Compressor Digital button
    if (SmartDashboard.getBoolean("Enable Compressor Digital", false)) {
    //SmartDashboard.putBoolean("Enable Compressor Digital", false);

    /**
    * Enable the compressor with digital sensor control.
    *
    * This will make the compressor run whenever the pressure switch is closed.
    * If open, (disconnected or reached max pressure), the compressor will shut
    * off.
    */
    m_ph.enableCompressorDigital();
    
    }

    // Disable Compressor button
    if (SmartDashboard.getBoolean("Disable Compressor", false)) {
    //SmartDashboard.putBoolean("Disable Compressor", false);
    m_ph.disableCompressor();
    /**
    * Disable the compressor.
    */
    }
    
    /**
     * Get the state of the solenoid.
     *
     * This is just a switch case to better display it on Shuffleboard.
     */
    switch (m_doubleSolenoid.get()) {
      case kOff:
        SmartDashboard.putString("Get Solenoid", "kOff");
        break;
      case kForward:
        SmartDashboard.putString("Get Solenoid", "kForward");
        break;
      case kReverse:
        SmartDashboard.putString("Get Solenoid", "kReverse");
        break;
      default:
        SmartDashboard.putString("Get Solenoid", "N/A");
        break;
    }

  }


//------------------------------------------------------------------------------------------------
  /** Autonomous Mode */
//------------------------------------------------------------------------------------------------
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }
//------------------------------------------------------------------------------------------------
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for half a second

   if (m_timer.get() < 0.3) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.8, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
   /* m_timer.restart();
    if (m_timer.get() < 1) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(-0.3, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
    m_timer.restart();
    if (m_timer.get() < 0.3) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.9, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    } */ 
    } 
//------------------------------------------------------------------------------------------------
  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    SmartDashboard.putString("Teleop", "Initialized!");
    SmartDashboard.putString("Pulley", "Init");
    SmartDashboard.putString("Set Solenoid", "Init");
  }

  public void doGripIn()
  {
        /** This is motor rotation code for the grip */
    if(m_timer.get() - lastGripIn > 10) {
    {
      lastGripIn = m_timer.get();
    }
    if (m_timer.get() - lastGripIn < 2) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_grip.set(0.1);
    }
    else {
      m_grip.stopMotor(); // stop robot
    }
    }
  }
  public void doGripOut()
  {
        /** This is motor rotation code for the grip */
    if(m_timer.get() - lastGripOut > 10) {
    {
      lastGripOut = m_timer.get();
    }
    if (m_timer.get() - lastGripOut < 2) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_grip.set(0.1);
    }
    else {
      m_grip.stopMotor(); // stop robot
    }
    }
  }

  public void doFlip()
  {
        /** This is motor rotation code for the grip */
    if(m_timer.get() - lastFlip > 10) {
    {
      lastFlip = m_timer.get();
    }
    if (m_timer.get() - lastFlip < 2) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_flip.set(0.1);
    }
    else {
      m_flip.stopMotor(); // stop robot
    }
    }
  }


//------------------------------------------------------------------------------------------------
  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX()); //two stick control
    //m_controller.axisGreaterThan(-m_controller.getRightX(), 0.2, boolean test);
    SmartDashboard.putNumber("left trigger access", m_controller.getLeftTriggerAxis());
    SmartDashboard.putNumber("right trigger axis", -m_controller.getRightTriggerAxis());
    SmartDashboard.putNumber("Not Right Stick (Left Stick)", (-m_controller.getLeftY()));
    SmartDashboard.putNumber("Not Left Stick (right stick)", (-m_controller.getRightX())); 

    if (m_controller.getLeftTriggerAxis()>0.5) {
      //m_grip.set(0.2);
      if(m_gripState) 
      {
        doGripOut();
        m_gripState = false;
      }
      else 
      {
        doGripIn();
        m_gripState = true;
      }
      // doGripIn();
    }
    /*else if (m_controller.getRightTriggerAxis()>0.5) { 
      m_grip.set(-0.2);
      // doGripOut();
    }*/
    /*else
    {
      m_grip.set(0);
    }*/

    if (m_controller.getYButton()) {
      m_flip.set(-0.2);
      // doFlip();
    }
    else
    {
      m_flip.set(0.0);
    }


    if (m_controller.getStartButton()) {
      m_ph.enableCompressorDigital();
    }

    if (m_controller.getBackButton()) {
      m_ph.disableCompressor();
    }
    // Set Off Button
    if (SmartDashboard.getBoolean("Set Off", false)) {
      SmartDashboard.putBoolean("Set Off", false);

      /**
       * Set the double solenoid to OFF.
       *
       * This will set both the forward and reverse solenoid channels to false.
       */
      m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);
    }
    
    if (m_controller.getBButtonPressed()) {
      SmartDashboard.putString("Set Solenoid", "Forward");
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (m_controller.getXButtonPressed()) {
      SmartDashboard.putString("Set Solenoid", "Reverse");
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    } else {
      SmartDashboard.putString("Set Solenoid", "Off");
      m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);
    }
  
    if (m_controller.getRightBumper()) {
      SmartDashboard.putString("Pulley", "Clockwise");
      // m_pulley.setInverted(true);
      m_pulley.set(0.6);
    } else if (m_controller.getLeftBumper()) {
      SmartDashboard.putString("Pulley", "Counter-clockwise");
      //m_pulley.setInverted(true);
      m_pulley.set(-1);
    } else {
      SmartDashboard.putString("Pulley", "Stop");
      m_pulley.stopMotor();
    }
  }
//------------------------------------------------------------------------------------------------
  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
