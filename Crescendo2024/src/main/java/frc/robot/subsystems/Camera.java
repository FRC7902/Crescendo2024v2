// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  
  private HttpCamera camera;
  private PhotonCamera driverCamera = new PhotonCamera("DriverCamera");
  private Mat mat;
  private CvSource outputStream;
  private CvSink cvSink;
  private Thread visionThread;

    public static final int resX = 80;
    public static final int resY = 80;

  // 0 < x < resX
  // 0 < y < resY
    public static final int[] rectPoint1 = {280, 80};
    public static final int[] rectPoint2 = {360, 120};

  //Rectangle colour

    public static final int[] rectBGR= {0, 0, 255};

  /** Creates a new Camera. */
  public Camera() {

    camera = new HttpCamera("DriverCamera", "http://photonvision.local:5800/");
    driverCamera.setDriverMode(true);
    camera.setResolution(resX, resY);
    cvSink = CameraServer.getVideo();

    outputStream = CameraServer.putVideo("Stream", resX, resY);

    mat = new Mat();

    visionThread = new Thread(
      () -> {
        while(!Thread.interrupted()){

          if(cvSink.grabFrame(mat) == 0){
            outputStream.notifyError(cvSink.getError());

            continue;
          }


          Imgproc.rectangle(mat, 
            new Point(rectPoint1[0], rectPoint1[1]), 
            new Point(rectPoint2[0], rectPoint2[1]), 
            new Scalar(rectBGR[0], rectBGR[1], rectBGR[2]));


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

