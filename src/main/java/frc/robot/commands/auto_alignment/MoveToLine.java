/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto_alignment;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class MoveToLine extends PIDCommand {
    public MoveToLine() {
        super(0.0024, 0, 0.001);
        requires(Robot.jetson);
        requires(Robot.driveTrain);
    }

    @Override
    protected double returnPIDInput() {
        return Robot.jetson.getYDistance();
    }

    @Override
    protected void usePIDOutput(double output) {
        System.out.println("PID output: " + output);
        Robot.driveTrain.drive(0, -output, 0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
