package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode; // VictorSPX
import com.ctre.phoenix6.signals.NeutralModeValue; // TalonFX

public class ShooterAndIntakeSubsystem extends SubsystemBase {
    private final VictorSPX cimMotor; // indexer/intake
    private final TalonFX kraken; // flywheel (Kraken X60)

    // Phoenix 6 closed-loop velocity request: onboard PID + (optional) feedforward
    // in volts.
    private final VelocityVoltage flywheelVelReq = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageReq = new VoltageOut(0);

    public ShooterAndIntakeSubsystem() {
        cimMotor = new VictorSPX(Constants.FuelConstants.IntakeID);
        kraken = new TalonFX(Constants.FuelConstants.IntakeUpID);

        // Indexer/intake generally likes Brake; flywheel usually likes Coast.
        cimMotor.setNeutralMode(NeutralMode.Brake);
        kraken.setNeutralMode(NeutralModeValue.Coast);

        cimMotor.setInverted(true);

        // Configure TalonFX onboard PID gains (Slot0)
        // TODO: tune these values (start small). Ideally use SysId.
        var cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = 0.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;

        kraken.getConfigurator().apply(cfg);
    }

    /**
     * Open-loop intake (kept from your original code).
     * If your Kraken is not part of intake, you can remove the kraken.set(...)
     * line.
     */
    public void intakeOn() {
        cimMotor.set(VictorSPXControlMode.PercentOutput, Constants.FuelConstants.IntakeSpeed);
        kraken.set(Constants.FuelConstants.IntakeSpeedUP);
    }

    public void intakeOff() {
        cimMotor.set(VictorSPXControlMode.PercentOutput, 0);
        kraken.set(0);
    }

    /**
     * Onboard velocity control (setpoint in RPM) with optional feedforward in
     * volts.
     */
    public void setFlywheelVelocityRPM(double targetRPM, double ffVolts) {
        double targetRPS = targetRPM / 60.0; // Phoenix 6 uses rotations/second
        kraken.setControl(
                flywheelVelReq.withVelocity(targetRPS)
                        .withFeedForward(ffVolts));
    }

    public void set() {
        kraken.set(0.6);
    }

    /** Convenience overload: no feedforward. */
    public void setFlywheelVelocityRPM(double targetRPM) {
        setFlywheelVelocityRPM(targetRPM, 0.0);
    }

    public void setFlywheelVoltage(double volts) {
        kraken.setControl(voltageReq.withOutput(volts));
    }

    public void stopFlywheel() {
        kraken.set(0);
    }

    public void setIndexerSpeed(double speed) {
        cimMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopIndexer() {
        setIndexerSpeed(0.0);
    }

    public double getFlyRPM() {
        return kraken.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("KrakenRPM", getFlyRPM());
        SmartDashboard.putNumber("KrakenRPS", kraken.getVelocity().getValueAsDouble());
    }
}