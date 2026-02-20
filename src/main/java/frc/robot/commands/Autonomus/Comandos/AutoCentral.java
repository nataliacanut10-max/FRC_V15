package frc.robot.commands.Autonomus.Comandos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveWithMeters;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.ShooterAndIntakeSubsystem;

public class AutoCentral  extends SequentialCommandGroup {
    public AutoCentral(CANDriveSubsystem drive, ShooterAndIntakeSubsystem shooter) {
        addCommands(
            //new Autoshoot(shooter, 5.0), 
            new DriveWithMeters(drive, 2.0)
            //new Shoot(shooter, 5000).withTimeout(5.0)
            //new DriveWithMeters(drive, 2.0)

            
        );
    }
}
