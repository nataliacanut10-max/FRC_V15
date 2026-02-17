package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BoxSubsystem;

public class MoveBox extends Command {

    private final BoxSubsystem box;
    private final double setpoint;
    private final PIDController pid;
    private final double tolerance = 1.5;

    public MoveBox(BoxSubsystem box, double setpoint) {
        this.box = box;
        this.setpoint = setpoint;
        pid = new PIDController(0.1, 0.0, 0.0);
        pid.setTolerance(tolerance);

        addRequirements(box);
    }

    @Override
    public void execute() {
        double leftV = box.getEncoder()[0];
        double rightV = box.getEncoder()[1];

        double speedR = pid.calculate(rightV, setpoint);
        double speedL = pid.calculate(leftV, setpoint);
        speedR = MathUtil.clamp(speedR, -0.5, .5);
        box.setSpeed(speedR, speedL);
    }

    @Override
    public void end(boolean interrupted) {
        box.setSpeed(0.0, 0.0);
    }
}
