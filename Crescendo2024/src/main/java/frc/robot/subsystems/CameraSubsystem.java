// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  Thread m_visionThread;
  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {
    m_visionThread = new Thread(
      // The thread improves efficency of multitasking: Photovision work synchronously with main
      () -> {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        // Automatic Capture
        camera.setResolution(100, 100);
        // Set intial resolution 

        CvSink cvSink = CameraServer.getVideo();
        // Puts video frames in mat element
        CvSource outputStream = CameraServer.putVideo("Rectangle", 100, 100);
        // Defines "rectangle" to send the video frames to the camera server
        
        Mat mat = new Mat();
        // Stores image data in pixel matrix

        while(!Thread.interrupted()){
          // Runs until thread interrupted
          if(cvSink.grabFrame(mat) == 0){
            // If there is no image in the camera it returns an error
            outputStream.notifyError(cvSink.getError());
            // Retrieves insight on image error
            continue;
            // continues to next iteration
          }
        }

        Imgproc.rectangle(mat, new Point (100,100), new Point(400,400), new Scalar(255, 255, 255), 5);
        // Rectangle drawn on mat's captured frames
        outputStream.putFrame(mat);
        // The new processed frame is sent to output stream

      }
    );
    m_visionThread.setDaemon(true);
    // The program can exit
    m_visionThread.start();
    


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
