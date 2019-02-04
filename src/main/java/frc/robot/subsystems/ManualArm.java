/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ManualArm extends Subsystem {
  private static WPI_TalonSRX backLeftLiftMotor = new WPI_TalonSRX(RobotMap.backLeftLiftMotor);
  private static WPI_TalonSRX backRightLiftMotor = new WPI_TalonSRX(RobotMap.backRightLiftMotor);
  private static WPI_TalonSRX frontLeftLiftMotor = new WPI_TalonSRX(RobotMap.frontLeftLiftMotor);
  private static WPI_TalonSRX frontRightLiftMotor = new WPI_TalonSRX(RobotMap.frontRightLiftMotor);

  private static Solenoid brake = new Solenoid(RobotMap.liftBrakeSolenoid);
  private static DoubleSolenoid upSolenoid = new DoubleSolenoid(RobotMap.liftUpSolenoidA, RobotMap.liftUpSolenoidB);
  private static DoubleSolenoid downSolenoid = new DoubleSolenoid(RobotMap.liftDownSolenoidA, RobotMap.liftDownSolenoidB);

  private static Encoder encoder = new Encoder(RobotMap.liftEncoderAPort, RobotMap.liftEncoderBPort);
  private static int ENCODER_PPR = 2048;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
