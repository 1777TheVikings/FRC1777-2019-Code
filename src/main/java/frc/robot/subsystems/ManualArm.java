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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopLift;

/**
 * Add your docs here.
 */
public class ManualArm extends Subsystem {
  private static WPI_TalonSRX leftLiftMotor = new WPI_TalonSRX(RobotMap.leftLiftMotor);
  private static WPI_TalonSRX rightLiftMotor = new WPI_TalonSRX(RobotMap.rightLiftMotor);


  private static Solenoid brake = new Solenoid(RobotMap.liftBrakeSolenoid);
  private static DoubleSolenoid upSolenoid = new DoubleSolenoid(RobotMap.liftUpSolenoidA, RobotMap.liftUpSolenoidB);
  private static DoubleSolenoid downSolenoid = new DoubleSolenoid(RobotMap.liftDownSolenoidA, RobotMap.liftDownSolenoidB);

  private static Encoder leftEncoder = new Encoder(RobotMap.liftLeftEncoderAPort, RobotMap.liftLeftEncoderBPort);
  private static Encoder rightEncoder = new Encoder(RobotMap.liftRightEncoderAPort, RobotMap.liftRightEncoderBPort);
  private static int ENCODER_PPR = 2048;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public ManualArm() {
    leftLiftMotor.configNeutralDeadband(0.1);
    
    rightLiftMotor.configNeutralDeadband(0.1);
    rightLiftMotor.setInverted(true);

    leftEncoder.setDistancePerPulse(360 / ENCODER_PPR);  // units are in degrees
    leftEncoder.setMinRate(2);

    rightEncoder.setDistancePerPulse(360 / ENCODER_PPR);  // units are in degrees
    rightEncoder.setMinRate(2);
    rightEncoder.setReverseDirection(true);

    setBrake(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new TeleopLift());
  }

  public void drive(double speed) {
    leftLiftMotor.set(speed);
    rightLiftMotor.set(speed);
  }

  public void setBrake(boolean on) {
    brake.set(on);
  }

  public void setSolenoids(Value value) {
    switch (value) {
      case kForward:
        upSolenoid.set(Value.kForward);
        downSolenoid.set(Value.kReverse);
        break;
      case kReverse:
        upSolenoid.set(Value.kReverse);
        downSolenoid.set(Value.kForward);
        break;
      case kOff:
      default:
        upSolenoid.set(Value.kForward);
        downSolenoid.set(Value.kForward);
        break;
    }
  }
}
