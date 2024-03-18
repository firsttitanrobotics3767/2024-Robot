package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Elevator extends SubsystemBase{
    public enum PositionState {
        STOW(0.5),
        AMP(20),
        SAFE_SHOOT(35);

        public double pos;
        private PositionState(double pos) {
            this.pos = pos;
        }
    }

    private static Elevator instance = null;
    
    private final TalonFX elevatorMotor;
    private final TalonFXConfiguration elevatorConfig;

    private PositionState goalState = PositionState.STOW;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }
    
    public Elevator() {
        elevatorMotor = new TalonFX(Constants.Elevator.motorID);
        elevatorConfig = new TalonFXConfiguration();
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 35;
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.2;
        elevatorMotor.getConfigurator().apply(elevatorConfig);
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor.setInverted(false);
        elevatorMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putString("Elevator/goalState", goalState.toString());
        // SmartDashboard.putNumber("Elevator/targetPos", goalState.pos);
        SmartDashboard.putNumber("Elevator/measuredPos", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/velocity", elevatorMotor.getVelocity().getValueAsDouble());
        // SmartDashboard.putBoolean("Elevator/at goal", atGoal());
    }

    public void moveTo(PositionState position) {
        goalState = position;
    }

    public void setSpeed(double speed) {
        elevatorMotor.set(speed * 0.5);
    }

    public double getPosition() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    public void resetPosition() {
        elevatorMotor.setPosition(0);
    }

    public boolean atGoal() {
        return ((getPosition() > (goalState.pos - 0.01)) && (getPosition() < (goalState.pos + 0.01)));
    }
}
