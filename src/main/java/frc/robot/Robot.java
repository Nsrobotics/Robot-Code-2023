// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  //private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  //private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final Talon m_leftDrive0 = new Talon(0);
  private final Talon m_leftDrive1 = new Talon(1);
  private final Talon m_rightDrive2 = new Talon(2);
  private final Talon m_rightDrive3 = new Talon(3);
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_leftDrive0,m_leftDrive1);
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightDrive2,m_rightDrive3);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
  private final AnalogGyro m_gyro = new AnalogGyro(0);
  Thread m_visionThread;
  private final DoubleSolenoid exampleDouble = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 7);
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
    m_gyro.reset();
    m_right.setInverted(true);
    m_left.setInverted(false);
  
      
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
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

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

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    m_robotDrive.arcadeDrive(0.5,0.50);
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      //m_robotDrive.arcadeDrive(0.25, 0.0, false);
      //m_robotDrive.arcadeDrive(0.25,0.50); 
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    SmartDashboard.putString("Teleop", "Initialized!");
   /*  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  phCompressor.enableDigital();
  phCompressor.disable();

  boolean enabled = phCompressor.enabled();
  boolean pressureSwitch = phCompressor.getPressureSwitchValue();
  double current = phCompressor.getCurrent();
  DoubleSolenoid exampleDoublePH = new DoubleSolenoid(9, PneumaticsModuleType.REVPH, 4, 5);

  exampleDoublePH.set(kOff);
  exampleDoublePH.set(kForward);
  exampleDoublePH.set(kReverse);
  */
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX()); //two stick control
    exampleDouble.set(DoubleSolenoid.Value.kForward);
    /*
    if (m_controller.getBButtonPressed()) {
      exampleDouble.toggle();
   }
   
else if (m_controller.getYButtonPressed()) {
  exampleDouble.toggle();
}


    exampleDouble.set(kReverse);
    if (m_controller.getXButtonPressed()) {
      exampleDouble.toggle();
   }
   */
    SmartDashboard.putNumber("Not Right Stick (Left Stick)", (-m_controller.getLeftY()));
    SmartDashboard.putNumber("Right Stick", (-m_controller.getRightX()));
    SmartDashboard.putNumber("Right Stick", (-m_controller.getRightX()));
    //SmartDashboard.putNumber("right motor speed", m_rightDrive.get());
    //SmartDashboard.putNumber("left motor speed", m_leftDrive.get());
    
    //m_robotDrive.arcadeDrive(-m_controller.getLeftY(), m_controller.getLeftX(),true); //This code is for a one control stick drive 
   // m_robotDrive.tankDrive(-m_controller.getLeftY(), -m_controller.getRightY()); // this code is for tank control
  }
  private void exampleDoubleset(Value kreverse) {
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}


