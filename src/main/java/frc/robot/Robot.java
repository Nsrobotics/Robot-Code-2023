// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Testing Change from my branch then to a Merge Request to Main Branch

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
import edu.wpi.first.wpilibj.AnalogGyro;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private static final int PH_CAN_ID = 11;
  private static int forwardChannel = 7;
  private static int reverseChannel = 6;
  PneumaticHub m_ph = new PneumaticHub(PH_CAN_ID);
  DoubleSolenoid m_doubleSolenoid = m_ph.makeDoubleSolenoid(forwardChannel,reverseChannel);
  
  private final XboxController m_controller = new XboxController(0);
  private final Talon m_leftDrive0 = new Talon(0);
  private final Talon m_leftDrive1 = new Talon(1);
  private final Talon m_rightDrive2 = new Talon(2);
  private final Talon m_rightDrive3 = new Talon(3);
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_leftDrive0,m_leftDrive1);
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightDrive2,m_rightDrive3);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);
  private final Timer m_timer = new Timer();
  private final AnalogGyro m_gyro = new AnalogGyro(0);
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
              camera.setResolution(480, 640);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Rectangle", 480, 640);

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
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
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
    SmartDashboard.putBoolean("Enable Compressor Digital", false);

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
    SmartDashboard.putBoolean("Disable Compressor", false);

    /**
    * Disable the compressor.
    */
    m_ph.disableCompressor();
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
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }
//------------------------------------------------------------------------------------------------
  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    SmartDashboard.putString("Teleop", "Initialized!");
  }
//------------------------------------------------------------------------------------------------
  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX()); //two stick control
    SmartDashboard.putNumber("left trigger access", m_controller.getLeftTriggerAxis());
    SmartDashboard.putNumber("right trigger axis", -m_controller.getRightTriggerAxis());
    SmartDashboard.putNumber("Not Right Stick (Left Stick)", (-m_controller.getLeftY()));
    SmartDashboard.putNumber("Not Left Stick (right stick)", (-m_controller.getRightX())); 
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

    // Set Forward button
    if (SmartDashboard.getBoolean("Set Forward", false)) {
      SmartDashboard.putBoolean("Set Forward", false);

      /**
       * Set the double solenoid direction to FORWARD.
       *
       * This will set the forward solenoid channel to true and the reverse
       * solenoid channel to false.
       */
    }
    
    if (m_controller.getBButtonPressed()) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (m_controller.getXButtonPressed()) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    } else {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);
    }
  

    // Set Reverse button
    if (SmartDashboard.getBoolean("Set Reverse", false)) {
      SmartDashboard.putBoolean("Set Reverse", false); 
      /**
       * Set the double solenoid direction to REVERSE.
       *
       * This will set the forward solenoid channel to false and the reverse
       * solenoid channel to true.
       */
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
