/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

import static frc.robot.subsystems.DualCamera.CameraPosition;

/**
 * Add your docs here.
 */
public class SwitchCamera extends InstantCommand {
  /**
   * Add your docs here.
   */
  public SwitchCamera() {
    super();
    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.dualCam);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (Robot.dualCam.currentPosition == CameraPosition.kFront) {
      Robot.dualCam.useBackCamera();
    } else {
      Robot.dualCam.useFrontCamera();
    }
  }
}
