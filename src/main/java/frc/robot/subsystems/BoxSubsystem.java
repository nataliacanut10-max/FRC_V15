package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class BoxSubsystem extends SubsystemBase {

    private SparkMax MotorLeft = new SparkMax(Constants.FuelConstants.BoxLeftID, MotorType.kBrushless);
    private SparkMax MotorRight = new SparkMax(Constants.FuelConstants.BoxRightID, MotorType.kBrushless);
    // private SparkMaxConfig motorConfig = new SparkMaxConfig();

    private RelativeEncoder EncoderLeft = MotorLeft.getEncoder();
    private RelativeEncoder EncoderRight = MotorRight.getEncoder();

    public BoxSubsystem() {
        /*
         * motorConfig.idleMode(IdleMode.kBrake);
         * 
         * Motor.configure(
         * motorConfig,
         * ResetMode.kResetSafeParameters,
         * PersistMode.kNoPersistParameters
         * );
         */
        EncoderLeft.setPosition(0);
        EncoderRight.setPosition(0);
    }

    public void stop() {
        MotorLeft.set(0);
        MotorRight.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Box Left", getEncoder()[0]);
        SmartDashboard.putNumber("Box Right", getEncoder()[1]);
    }

    public double[] getEncoder() {
        double[] gets = new double[2];
        gets[0] = EncoderLeft.getPosition() * -10;
        gets[1] = EncoderRight.getPosition() * 10;
        return gets;
    }

    public void setSpeed(double right, double left) {
        MotorLeft.set(left);
        MotorRight.set(-right);
    }

    public void resetBox(){
        EncoderLeft.setPosition(0);
        EncoderRight.setPosition(0);
    }
}