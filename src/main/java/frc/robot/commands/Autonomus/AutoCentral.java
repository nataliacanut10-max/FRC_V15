package frc.robot.commands.Autonomus;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveWithMeters;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.ShooterAndIntakeSubsystem;
import frc.robot.subsystems.Vision;

public class AutoCentral extends SequentialCommandGroup {
    public AutoCentral(CANDriveSubsystem drive, ShooterAndIntakeSubsystem shooter, Vision vision) {
        addCommands(
                new Shoot(shooter, 3500),
                new DriveWithMeters(drive, 2.0));
    }
}