package frc.robot;

import java.io.OutputStream;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  private Thread visionThread;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit(){
    try {
      visionThread = new Thread(() -> {
        // Start the camera
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);

        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Processed", 640, 480);
        Mat frame = new Mat();
        Shuffleboard.getTab("Vision").add("Processed", outputStream);
        

        while (!Thread.interrupted()) {
          if (cvSink.grabFrame(frame) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
          }

          // Draw two vertical lines
          int x1 = frame.width() / 3; // First line at 1/3 of the width
          int x2 = 2 * frame.width() / 3; // Second line at 2/3 of the width

          Imgproc.line(frame, new Point(x1, 0), new Point(x1, frame.height()), new Scalar(0, 255, 0), 2);
          Imgproc.line(frame, new Point(x2, 0), new Point(x2, frame.height()), new Scalar(0, 255, 0), 2);

          outputStream.putFrame(frame);
        }
      });

      visionThread.setDaemon(true);
      visionThread.start();
      
      
    } catch (Exception e) {
      System.out.println("Error in vision thread");
    }
}
  

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }      
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
