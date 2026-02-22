// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimWithLL;
import frc.robot.commands.DriveWithMeters;
import frc.robot.commands.MoveBox;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootHub;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.Autonomus.AutoCentral;
import frc.robot.subsystems.BoxSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterAndIntakeSubsystem;
import frc.robot.subsystems.Vision;

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
  private final Vision vision;

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final CommandXboxController m_controller2 = new CommandXboxController(1);

  public RobotContainer() {
    // Subsistemas
    driveSubsystem = new CANDriveSubsystem();

    shooterAndIntake = new ShooterAndIntakeSubsystem();

    boxSubsystem = new BoxSubsystem();

    climberSubsystem = new ClimberSubsystem();

    vision = new Vision();

    setAutoOptions();
    configureBindings();
    SmartDashboard.putData("Auto Centro", m_chooser);
  }

  private void configureBindings() {
    // Movimentação
    driveSubsystem.setDefaultCommand(new TankDriveCommand(driveSubsystem, () -> m_controller.getLeftY(),
        () -> m_controller.getRightX(), Constants.DriveConstants.MAX_SPEED));

    m_controller.leftTrigger().toggleOnTrue(new TankDriveCommand(driveSubsystem, () -> m_controller.getLeftY(),
        () -> m_controller.getRightX(), Constants.DriveConstants.SLOW_SPEED));
    // Collect
    m_controller2.a()
        .whileTrue(Commands.startEnd(() -> shooterAndIntake.intakeOn(), () -> shooterAndIntake.intakeOff()));

    // ShortShoot
    m_controller2.x().whileTrue(new ShootHub(shooterAndIntake, vision, 3500));

    // MoveBox
    m_controller2.rightBumper().whileTrue(new MoveBox(boxSubsystem,
        Constants.BoxConstants.extendedSet));
    m_controller2.leftBumper().whileTrue(new MoveBox(boxSubsystem,
        Constants.BoxConstants.retractSet));
    m_controller2.b().and(m_controller2.y()).onTrue(Commands.runOnce(() -> boxSubsystem.resetBox()));

    m_controller.b().whileTrue(new DriveWithMeters(driveSubsystem, 1));
    m_controller.x().whileTrue(new DriveWithMeters(driveSubsystem, 0));
    m_controller.a().and(m_controller.y()).onTrue(Commands.runOnce(() -> driveSubsystem.resetEncoders()));

    // Aim the robot to shoot
    m_controller.b().whileTrue(new AimWithLL(driveSubsystem, vision));

    // Climber
    m_controller.y()
        .whileTrue(Commands.startEnd(() -> climberSubsystem.climbOn(-0.5), () -> climberSubsystem.climbOn(0)));
    m_controller.a()
        .whileTrue(Commands.startEnd(() -> climberSubsystem.climbOn(0.5), () -> climberSubsystem.climbOn(0)));
  }

  public void setAutoOptions() {
    m_chooser.setDefaultOption("Auto Centro", new AutoCentral(driveSubsystem, shooterAndIntake, vision));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
