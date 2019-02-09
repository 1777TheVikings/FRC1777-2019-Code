/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import frc.robot.RobotMap;
import frc.robot.commands.TeleopDrive;
import frc.utils.SelfCheckError;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static VictorSP frontLeftMotor = new VictorSP(RobotMap.frontLeftMotor);
  private static VictorSP frontRightMotor = new VictorSP(RobotMap.frontRightMotor);
  private static VictorSP backLeftMotor = new VictorSP(RobotMap.backLeftMotor);
  private static VictorSP backRightMotor = new VictorSP(RobotMap.backRightMotor);
  
  private static MecanumDrive robotDrive;

  private static PigeonIMU pigeon = new PigeonIMU(RobotMap.pigeon);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new TeleopDrive());
    robotDrive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
  }

  public void selfCheck() throws SelfCheckError {
    if (pigeon.getFirmwareVersion() == 0)
      throw new SelfCheckError("Pigeon IMU with ID " + String.valueOf(RobotMap.pigeon) + " is disconnected");
  }

  public void drive(double x, double y, double turn) {
    robotDrive.driveCartesian(y, x, turn);
    DriverStation.reportWarning("drive 2", false);
  }

  public double getAngle() {
    double[] ypr_deg = new double[3];
    pigeon.getYawPitchRoll(ypr_deg);
    return ypr_deg[0];
  }

  public void tarePigeon() {
    pigeon.setYaw(0.0);
  }
}
