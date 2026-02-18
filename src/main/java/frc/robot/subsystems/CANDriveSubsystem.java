package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class CANDriveSubsystem extends SubsystemBase {
  private final VictorSPX leftLeader, leftFollower, rightLeader, rightFollower;
  private final Encoder rightEncoder, leftEncoder;

  private final PIDController rightPid = new PIDController(
      DriveConstants.kP,
      DriveConstants.kI,
      DriveConstants.kD);

  private final PIDController leftPid = new PIDController(
      DriveConstants.kP,
      DriveConstants.kI,
      DriveConstants.kD);

  private final double wheelCircunference = Math.PI * 2 * Units.inchesToMeters(3);
  private final double ticksPerRev = 2048;

  public CANDriveSubsystem() {

    rightEncoder = new Encoder(0, 1, true, EncodingType.k4X);
    leftEncoder = new Encoder(2, 3, true, EncodingType.k4X);

    leftLeader = new VictorSPX(Constants.DriveConstants.LEFT_LEADER_ID);
    leftFollower = new VictorSPX(Constants.DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new VictorSPX(Constants.DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new VictorSPX(Constants.DriveConstants.RIGHT_FOLLOWER_ID);

    leftLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
    leftLeader.setInverted(true);
    leftFollower.setInverted(true);

    leftEncoder.reset();
    rightEncoder.reset();
    // leftEncoder.setDistancePerPulse(wheelCircunference / 8192);
    // rightEncoder.setDistancePerPulse(wheelCircunference / 8192);
  }

  public double getLeftMeters() {
    return getLeftEncoder() / ticksPerRev * wheelCircunference;
  }

  public double getRightMeters() {
    return getRightEncoder() / ticksPerRev * wheelCircunference;
  }

  public double getRightEncoder() {
    return rightEncoder.get();
  }

  public double getLeftEncoder() {
    return leftEncoder.get();
  }

  public double getRightVelocity() {
    return rightEncoder.getRate();
  }

  public double getLeftVelocity() {
    return leftEncoder.getRate();
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void setMotorVoltage(double leftVolts, double rightVolts) {
    double battery = RobotController.getBatteryVoltage();

    double leftPercent = MathUtil.clamp(leftVolts / battery, -1, 1);
    double rightPercent = MathUtil.clamp(rightVolts / battery, -1, 1);

    leftLeader.set(ControlMode.PercentOutput, leftPercent);
    leftFollower.set(ControlMode.PercentOutput, leftPercent);

    rightLeader.set(ControlMode.PercentOutput, rightPercent);
    rightFollower.set(ControlMode.PercentOutput, rightPercent);
  }

  public void drive(double leftSide, double rightSide) {
    rightLeader.set(ControlMode.PercentOutput, rightSide);
    rightFollower.set(ControlMode.PercentOutput, rightSide);

    leftLeader.set(ControlMode.PercentOutput, leftSide);
    leftFollower.set(ControlMode.PercentOutput, leftSide);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drive Left", getLeftEncoder());
    SmartDashboard.putNumber("Drive Right", getRightEncoder());

    double left = getLeftMeters();
    double right = getRightMeters();

    SmartDashboard.putNumber("Drive Left in Meter", left);
    SmartDashboard.putNumber("Drive Right in Meter", right);
  }

}
