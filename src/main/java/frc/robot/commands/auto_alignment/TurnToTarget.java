/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto_alignment;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class TurnToTarget extends PIDCommand {
    private double startAngle;
    private double endAngle;

    public TurnToTarget() {
        super(0.1, 0.0, 0.0);
        requires(Robot.driveTrain);
        setTimeout(1.0);  // failsafe
        startAngle = Robot.driveTrain.getAngle();
        endAngle = startAngle + Robot.jetson.getAngle();
        setSetpoint(endAngle);
    }

    @Override
    protected double returnPIDInput() {
        return Robot.driveTrain.getAngle();
    }

    @Override
    protected void usePIDOutput(double output) {
        Robot.driveTrain.drive(0, 0, output);
    }

    @Override
    protected boolean isFinished() {
        if (Math.abs(getSetpoint() - getPosition()) < 0.4)
            return true;
        if (isTimedOut()){
            DriverStation.reportWarning("TurnToTarget timed out", false);
            return true;
        }
        return false;
    }
}
