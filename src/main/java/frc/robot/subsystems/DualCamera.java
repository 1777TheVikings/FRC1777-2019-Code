/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DualCamera extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static UsbCamera backCamera;
  private static UsbCamera frontCamera;
  private VideoSink server;
  public CameraPosition currentPosition = CameraPosition.kFront;

  public static enum CameraPosition {
    kFront, kBack
  }

  @Override
  public void initDefaultCommand() {
  

    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public DualCamera() {
    frontCamera = CameraServer.getInstance().startAutomaticCapture(0);
    backCamera = CameraServer.getInstance().startAutomaticCapture(1);
    // frontCamera = new UsbCamera("Front camera", RobotMap.frontCamera);
    // backCamera = new UsbCamera("Back camera", RobotMap.backCamera);
    server = CameraServer.getInstance().getServer();
    frontCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
    backCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    useFrontCamera();
  }



  public void useFrontCamera() {
    server.setSource(frontCamera);
    currentPosition = CameraPosition.kFront;
  }
  public void useBackCamera() {
    server.setSource(backCamera);
    currentPosition = CameraPosition.kBack;
  }
}
