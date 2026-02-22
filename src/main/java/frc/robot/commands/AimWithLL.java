package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.Vision;

public class AimWithLL extends Command {
    private PIDController rotatePID, translationPID;
    private CANDriveSubsystem drive;
    private Vision vision;
    private double distance, yaw;

    public AimWithLL(CANDriveSubsystem drive, Vision vision) {
        this.drive = drive;
        this.vision = vision;
        rotatePID = new PIDController(1, 0, 0);
        rotatePID.enableContinuousInput(-Math.PI, Math.PI);
        rotatePID.setTolerance(1.0);
        translationPID = new PIDController(1, 0, 0);
        translationPID.setTolerance(0.1);

        addRequirements(drive);
    }

    @Override
    public void execute() {
        yaw = Units.degreesToRadians(vision.getTX());
        distance = vision.getDistance();
        if (vision.hasTarget()) {
            double rotateSpeed = rotatePID.calculate(yaw, 0.0);
            rotateSpeed = MathUtil.clamp(rotateSpeed, -.5, .5);
            drive.drive(rotateSpeed, -rotateSpeed);
            if (rotatePID.atSetpoint()) {
                double translationSpeed = translationPID.calculate(distance, CameraConstants.distanceToShoot);
                translationSpeed = MathUtil.clamp(translationSpeed, -.5, .5);
                drive.drive(-translationSpeed, translationSpeed);
            }
        }
        drive.drive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return rotatePID.atSetpoint() && translationPID.atSetpoint() ? true : false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0);
    }

}
