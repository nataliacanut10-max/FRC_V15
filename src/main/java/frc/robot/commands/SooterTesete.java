package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterAndIntakeSubsystem;

public class SooterTesete extends SequentialCommandGroup {

    public SooterTesete(ShooterAndIntakeSubsystem subsystem) {
        addCommands(Commands.runOnce(() -> subsystem.set()), new WaitCommand(3),
                Commands.runOnce(() -> subsystem.setIndexerSpeed(1)));
    }
}
