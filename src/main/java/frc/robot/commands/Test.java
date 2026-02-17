package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAndIntakeSubsystem;

public class Test extends Command {
    ShooterAndIntakeSubsystem subsystem;

    public Test(ShooterAndIntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double volts = SmartDashboard.getNumber("Flywheel kS", 0);
        subsystem.setFlywheelVoltage(volts);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setFlywheelVoltage(0);
    }
}
