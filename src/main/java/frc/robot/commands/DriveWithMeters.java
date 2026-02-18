package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

public class DriveWithMeters extends Command {
    private CANDriveSubsystem subsystem;
    private double meters;

    private PIDController pidRight, pidLeft;

    public DriveWithMeters(CANDriveSubsystem subsystem, double meters) {
        this.subsystem = subsystem;
        this.meters = meters;
        pidRight = new PIDController(2, 0, 0.);
        pidLeft = new PIDController(2, 1, 0.0);
        double tolerance = meters * 0.01;
        // pidRight.setTolerance(tolerance);
        // pidLeft.setTolerance(tolerance);

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.resetEncoders();
    }

    @Override
    public void execute() {

        double leftMeters = subsystem.getLeftMeters();
        double rightMeters = subsystem.getRightMeters();

        double leftSpeed = pidLeft.calculate(leftMeters, meters);
        double rightSpeed = pidRight.calculate(rightMeters, meters);
        leftSpeed = MathUtil.clamp(leftSpeed, -0.2, 0.2);
        rightSpeed = MathUtil.clamp(rightSpeed, -0.2, 0.2);

        subsystem.drive(-leftSpeed, -leftSpeed);

        SmartDashboard.putNumber("Error PID", pidRight.getError());
    }

    @Override
    public boolean isFinished() {
        return pidRight.atSetpoint() && pidLeft.atSetpoint() ? true : false;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.drive(0.0, 0.0);
    }

}
