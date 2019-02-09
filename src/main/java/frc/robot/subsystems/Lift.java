/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;
import frc.utils.SelfCheckError;

/**
 * Add your docs here.
 */
public class Lift extends PIDSubsystem {
  private static WPI_TalonSRX leftLiftMotor = new WPI_TalonSRX(RobotMap.leftLiftMotor);
  private static WPI_TalonSRX rightLiftMotor = new WPI_TalonSRX(RobotMap.rightLiftMotor);

  private static Solenoid brake = new Solenoid(RobotMap.liftBrakeSolenoid);
  private static DoubleSolenoid upSolenoid = new DoubleSolenoid(RobotMap.liftUpSolenoidA, RobotMap.liftUpSolenoidB);
  private static DoubleSolenoid downSolenoid = new DoubleSolenoid(RobotMap.liftDownSolenoidA, RobotMap.liftDownSolenoidB);

  private static Encoder leftEncoder = new Encoder(RobotMap.liftLeftEncoderAPort, RobotMap.liftLeftEncoderBPort);
  private static Encoder rightEncoder = new Encoder(RobotMap.liftRightEncoderAPort, RobotMap.liftRightEncoderBPort);
  private static int ENCODER_PPR = 2048;

  public Lift() {
    // Intert a subsystem name and PID values here
    super("Lift", 1, 2, 3);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    rightLiftMotor.setInverted(true);

    leftEncoder.setDistancePerPulse(360 / ENCODER_PPR);  // units are in degrees
    leftEncoder.setMinRate(2);

    rightEncoder.setDistancePerPulse(360 / ENCODER_PPR);  // units are in degrees
    rightEncoder.setMinRate(2);
    rightEncoder.setReverseDirection(true);

    setAbsoluteTolerance(3);
    setSetpoint(0.0);
    setBrake(true);
    disable();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void selfCheck() throws SelfCheckError {
    if (leftLiftMotor.getFirmwareVersion() == 0)
      throw new SelfCheckError("Talon SRX with ID " + RobotMap.leftLiftMotor + " is disconnected");
    if (rightLiftMotor.getFirmwareVersion() == 0)
      throw new SelfCheckError("Talon SRX with ID " + RobotMap.rightLiftMotor + " is disconnected");

  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return (leftEncoder.get() + rightEncoder.get()) / 2.0;
  }

  @Override
  protected void usePIDOutput(double output) {
   leftLiftMotor.pidWrite(output);
   rightLiftMotor.pidWrite(output);
  }

  public void tareEncoder() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void setBrake(boolean enableBrake) {
    brake.set(enableBrake);
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
