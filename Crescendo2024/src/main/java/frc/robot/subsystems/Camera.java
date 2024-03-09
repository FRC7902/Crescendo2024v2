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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  
  private UsbCamera camera;
  private Mat mat;
  private CvSource outputStream;
  private CvSink cvSink;
  private Thread visionThread;

  /** Creates a new Camera. */
  public Camera() {

    camera = CameraServer.startAutomaticCapture(0);
    camera.setResolution(Constants.CameraConstants.resX, Constants.CameraConstants.resY);

    cvSink = CameraServer.getVideo();

    outputStream = CameraServer.putVideo("Stream", Constants.CameraConstants.resX, Constants.CameraConstants.resY);

    mat = new Mat();

    visionThread = new Thread(
      () -> {
        while(!Thread.interrupted()){

          if(cvSink.grabFrame(mat) == 0){
            outputStream.notifyError(cvSink.getError());

            continue;
          }


          Imgproc.rectangle(mat, 
            new Point(Constants.CameraConstants.rectPoint1[0], Constants.CameraConstants.rectPoint1[1]), 
            new Point(Constants.CameraConstants.rectPoint2[0], Constants.CameraConstants.rectPoint2[1]), 
            new Scalar(Constants.CameraConstants.rectBGR[0], Constants.CameraConstants.rectBGR[1], Constants.CameraConstants.rectBGR[2]));


          outputStream.putFrame(mat);
        }
      }
    );

    visionThread.setDaemon(true);
    visionThread.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
