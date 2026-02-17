package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class CANDriveSubsystem extends SubsystemBase {
  private final VictorSPX leftLeader;
  private final VictorSPX leftFollower;
  private final VictorSPX rightLeader;
  private final VictorSPX rightFollower;
  private final Encoder rightEncoder, leftEncoder;

  private final PIDController rightPid = new PIDController(
      DriveConstants.kP,
      DriveConstants.kI,
      DriveConstants.kD);

  private final PIDController leftPid = new PIDController(
      DriveConstants.kP,
      DriveConstants.kI,
      DriveConstants.kD);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      DriveConstants.kS,
      DriveConstants.kV,
      DriveConstants.kA);

  // Tank Objects
  // private final DifferentialDriveOdometry odometry;
  private final DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(26.5));

  private RobotConfig config;

  private final double distancePerRevoltionInMeter = Math.PI * Units.inchesToMeters(3) * 2;

  private Field2d field = new Field2d();

  public CANDriveSubsystem() {

    rightEncoder = new Encoder(0, 1, true, EncodingType.k4X);
    leftEncoder = new Encoder(2, 3, true, EncodingType.k4X);

    leftLeader = new VictorSPX(Constants.DriveConstants.LEFT_LEADER_ID);
    leftFollower = new VictorSPX(Constants.DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new VictorSPX(Constants.DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new VictorSPX(Constants.DriveConstants.RIGHT_FOLLOWER_ID);

    leftLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    leftLeader.setInverted(true);
    leftFollower.setInverted(true);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftEncoder.reset();
    rightEncoder.reset();
    leftEncoder.setDistancePerPulse(distancePerRevoltionInMeter / 8192);
    rightEncoder.setDistancePerPulse(distancePerRevoltionInMeter / 8192);

    // odometry = new DifferentialDriveOdometry(getGyro(),
    // getWheelPositions().leftMeters,
    // getWheelPositions().rightMeters);

    // try {
    // config = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    // // Handle exception as needed
    // e.printStackTrace();
    // }

    // // Configure AutoBuilder last
    // AutoBuilder.configure(
    // this::getPose, // Robot pose supplier
    // this::resetPose, // Method to reset odometry (will be called if your auto has
    // a starting pose)
    // this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
    // RELATIVE
    // (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will
    // drive the robot given ROBOT RELATIVE
    // // ChassisSpeeds. Also optionally outputs individual
    // // module feedforwards
    // new PPLTVController(0.02), // PPLTVController is the built in path following
    // controller for differential
    // // drive trains
    // config, // The robot configuration
    // () -> {
    // // Boolean supplier that controls when the path will be mirrored for the red
    // // alliance
    // // This will flip the path being followed to the red side of the field.
    // // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    // var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    // return alliance.get() == DriverStation.Alliance.Red;
    // }
    // return false;
    // },
    // this // Reference to this subsystem to set requirements
    // );
  }

  public double getRightEncoder() {
    return rightEncoder.getDistance();
  }

  public double getLeftEncoder() {
    return leftEncoder.getDistance();
  }

  public double getRightVelocity() {
    return rightEncoder.getRate();
  }

  public double getLeftVelocity() {
    return leftEncoder.getRate();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return differentialDriveKinematics.toChassisSpeeds(getWheelSpeeds());
  }

  // public DifferentialDriveOdometry getOdometry() {
  // return odometry;
  // }

  // public Pose2d getPose() {
  // return odometry.getPoseMeters();
  // }

  // public void resetPose(Pose2d initialPose) {
  // resetEncoders();
  // odometry.resetPosition(getGyro(), getWheelPositions(), initialPose);
  // }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void calculateWithMPS(double leftMPS, double rightMPS) {
    double rightSpeed = rightPid.calculate(getRightVelocity(), rightMPS)
        + feedforward.calculate(rightMPS);

    double leftSpeed = leftPid.calculate(getLeftVelocity(), leftMPS)
        + feedforward.calculate(leftMPS);

    setMotorVoltage(leftSpeed, rightSpeed);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = differentialDriveKinematics.toWheelSpeeds(speeds);

    calculateWithMPS(
        wheelSpeeds.leftMetersPerSecond,
        wheelSpeeds.rightMetersPerSecond);
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

  public DifferentialDriveWheelPositions getWheelPositions() {
    return new DifferentialDriveWheelPositions(
        leftEncoder.getDistance(),
        rightEncoder.getDistance());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftEncoder.getRate(),
        rightEncoder.getRate());
  }

  @Override
  public void periodic() {
    // odometry.update(getGyro(), getWheelPositions());
    // field.setRobotPose(getPose());
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("RightSpeed", getWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("LeftSpeed", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("Drive Left", getLeftEncoder());
    SmartDashboard.putNumber("Drive Right", getRightEncoder());
    // SmartDashboard.putNumber("Gyro", getGyro().getDegrees());
  }

}
