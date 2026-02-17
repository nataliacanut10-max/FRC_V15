package frc.robot.commands.Autonomus.Comandos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

public class Autodrivebackwards extends Command {
    private final CANDriveSubsystem CANDriveSubsystem;
    private final Timer timer;
    private final double tempo;

    public Autodrivebackwards(CANDriveSubsystem CANDriveSubsystem, double tempo) {
        this.CANDriveSubsystem = CANDriveSubsystem;
        this.timer = new Timer();
        this.tempo = tempo;

        addRequirements(CANDriveSubsystem); // Requer o subsistema de drive
    }

    @Override
    public void initialize() {
        timer.reset(); // Reseta o timer
        timer.start(); // Inicia o timer
    }

    @Override
    public void execute() {
        CANDriveSubsystem.drive(0.25, 0.25);
    }

    @Override
    public void end(boolean interrupted) {
        CANDriveSubsystem.drive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(tempo); // Tempo que o robo vai andar pra tras
    }
}