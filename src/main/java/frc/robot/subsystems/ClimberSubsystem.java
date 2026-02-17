package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final VictorSPX climber = new VictorSPX(Constants.ClimberConstants.climberID);

    public ClimberSubsystem() {

    }

    public void climbOn(double speed) {
        climber.set(ControlMode.PercentOutput, speed);
    }
}
