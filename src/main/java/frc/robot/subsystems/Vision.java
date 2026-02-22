package frc.robot.subsystems;

import java.io.IOException;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    private String cameraName = "mapleCam";

    private final AprilTagFieldLayout fieldLayout;

    private double llAngleMount = CameraConstants.llAngleMount;
    private double llHeightToGndInches = Units.metersToInches(CameraConstants.llHeight);
    private double hubHeight = 44.25;
    private double shootTolerance = 0.1;
    private double yawTolerance = 1;

    public Vision() {
        try {
            fieldLayout = new AprilTagFieldLayout(
                    Filesystem.getDeployDirectory()
                            .toPath()
                            .resolve("apriltags/2026-official.json"));
        } catch (IOException e) {
            throw new RuntimeException("Falha ao carregar AprilTag map", e);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Camera TX", getTX());
        SmartDashboard.putNumber("Camera TY", getTY());
        SmartDashboard.putNumber("Camera Distance", getDistance());
        SmartDashboard.putBoolean("Ready To Shoot", readyToShoot());
    }

    public double getTX() {
        return LimelightHelpers.getTX(cameraName);
    }

    public double getTY() {
        return LimelightHelpers.getTY(cameraName);
    }

    public double getDistance() {
        double ty = getTY();

        if (hubHeight == 0.0) {
            hubHeight = 44.25;
        }

        double angleToGoalRadians = Units.degreesToRadians(llAngleMount + ty);

        double distanceFromLimelightToGoalInches = (hubHeight - llHeightToGndInches)
                / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(cameraName);
    }

    public boolean readyToShoot() {
        boolean distance = Math.abs(getDistance() - CameraConstants.distanceToShoot) <= shootTolerance;

        boolean yaw = Math.abs(getTX() - 0.0) <= yawTolerance;

        return distance && yaw;
    }
}
