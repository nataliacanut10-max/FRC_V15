// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveWithMeters;
import frc.robot.commands.MoveBox;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterTeste;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.BoxSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterAndIntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem;
  private final ShooterAndIntakeSubsystem shooterAndIntake;
  private final BoxSubsystem boxSubsystem;
  private final ClimberSubsystem climberSubsystem;

  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final CommandXboxController m_controller2 = new CommandXboxController(1);

  // The autonomous chooser
  // private final SendableChooser<Command> m_autoChooser;

  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Subsistemas
    driveSubsystem = new CANDriveSubsystem();

    shooterAndIntake = new ShooterAndIntakeSubsystem();

    boxSubsystem = new BoxSubsystem();

    climberSubsystem = new ClimberSubsystem();

    configureBindings();

    // m_autoChooser.setDefaultOption("1 - [AUTO C]", new
    // Autocenter(m_shooterSubsystem, m_CanDriveSubsystem));
    // m_autoChooser.addOption("2 - [AUTO 1]", new AutoLeftOne(m_shooterSubsystem,
    // dtRight2, dtRight1, dtLeft2, dtLeft1));
    // m_autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", m_autoChooser);

  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new TankDriveCommand(driveSubsystem, () -> m_controller.getLeftY(),
        () -> m_controller.getRightX(), Constants.DriveConstants.MAX_SPEED));

    // m_controller.rightTrigger()
    // .whileTrue(Commands.runOnce(() -> driveSubsystem.calculateWithMPS(1200,
    // 1200)));

    m_controller.leftTrigger().toggleOnTrue(new TankDriveCommand(driveSubsystem, () -> m_controller.getLeftY(),
        () -> m_controller.getRightX(), Constants.DriveConstants.SLOW_SPEED));

    // Collect
    m_controller2.a()
        .whileTrue(Commands.startEnd(() -> shooterAndIntake.intakeOn(), () -> shooterAndIntake.intakeOff()));

    // ShortShoot
    m_controller2.x().whileTrue(new Shoot(shooterAndIntake, 3500));

    // MoveBox
    m_controller2.rightBumper().whileTrue(new MoveBox(boxSubsystem,
        Constants.BoxConstants.extendedSet));
    m_controller2.leftBumper().whileTrue(new MoveBox(boxSubsystem,
        Constants.BoxConstants.retractSet));
    m_controller2.b().and(m_controller2.y()).onTrue(Commands.runOnce(() -> boxSubsystem.resetBox()));

    m_controller.b().whileTrue(new DriveWithMeters(driveSubsystem, 1));
    m_controller.x().whileTrue(new DriveWithMeters(driveSubsystem, 0));
    m_controller.a().and(m_controller.y()).onTrue(Commands.runOnce(() -> driveSubsystem.resetEncoders()));

    // climberSubsystem.climbOn(0)));

    // m_controller.y()
    // .whileTrue(Commands.startEnd(() -> climberSubsystem.climbOn(-0.5), () ->
    // climberSubsystem.climbOn(0)));

    // m_controller2.x().whileTrue(new
    // ShooterTeste(shooterAndIntake)).whileFalse(Commands.runOnce(() -> {
    // shooterAndIntake.stopFlywheel();
    // shooterAndIntake.stopIndexer();
    // }));
  }

  public Command getAutonomousCommand() {
    return new MoveBox(boxSubsystem, Constants.BoxConstants.extendedSet);
    // return m_autoChooser.getSelected();
  }
}
