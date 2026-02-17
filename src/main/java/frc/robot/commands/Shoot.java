package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAndIntakeSubsystem;

public class Shoot extends Command {
     private final ShooterAndIntakeSubsystem subsystem;
    private final double desiredRPM;

    // Feedforward in VOLTS. Units depend on the velocity unit you feed in.
    // Here we use RPS (rotations/second), because TalonFX velocity setpoint is RPS.
    // Tune these with SysId (recommended) or empirically.
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
            0.4,  // kS (volts)
            0.0,  // kV (volts per RPS)  <-- placeholder
            0.03  // kA (volts per RPS/s) <-- placeholder
    );

    private static final double INDEXER_POWER = 1.0;
    private static final double RPM_TOLERANCE = 150.0; // start 150~250 and adjust
    private static final double MAX_FF_VOLTS = 12.0;

    public Shoot(ShooterAndIntakeSubsystem subsystem, double desiredRPM) {
        this.subsystem = subsystem;
        this.desiredRPM = desiredRPM;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopIndexer();
    }

    @Override
    public void execute() {
        // Feedforward (optional but recommended)
        double targetRPS = desiredRPM / 60.0;
        double ffVolts = ff.calculate(targetRPS);
        ffVolts = MathUtil.clamp(ffVolts, -MAX_FF_VOLTS, MAX_FF_VOLTS);

        // Onboard PID (TalonFX Slot0) + FF volts
        subsystem.setFlywheelVelocityRPM(desiredRPM, ffVolts);

        // Feed note only when we're within tolerance
        double currentRPM = subsystem.getFlyRPM();
        boolean ready = Math.abs(currentRPM - desiredRPM) <= RPM_TOLERANCE;

        if (ready) {
            subsystem.setIndexerSpeed(INDEXER_POWER);
        } else {
            subsystem.stopIndexer();
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopFlywheel();
        subsystem.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
