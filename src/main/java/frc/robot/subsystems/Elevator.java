package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Elevator extends SubsystemBase{
    public enum PositionState {
        STOW(1),
        AMP(33),
        SAFE_SHOOT(35);

        public double pos;
        private PositionState(double pos) {
            this.pos = pos;
        }
    }

    private static Elevator instance = null;
    
    private final TalonFX elevatorMotor;
    private final TalonFXConfiguration elevatorConfig;

    private double finalSpeed;

    private boolean positionControlled = true;
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
        elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 30;
        elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 16;
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor.setInverted(false);
        elevatorMotor.setPosition(0);
        elevatorConfig.Slot0.kP = Constants.Elevator.positionP;
        elevatorConfig.Slot0.kI = Constants.Elevator.positionI;
        elevatorConfig.Slot0.kD = Constants.Elevator.positionD;
        elevatorConfig.Slot0.kG = Constants.Elevator.positionG;
        elevatorConfig.Slot0.kV = Constants.Elevator.positionV;
        elevatorConfig.Slot0.kS = Constants.Elevator.positionS;
        elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Elevator.maxVel;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Elevator.maxAccel;

        elevatorMotor.getConfigurator().apply(elevatorConfig);
        elevatorMotor.getConfigurator().apply(motionMagicConfigs);
    }

    @Override
    public void periodic() {
        // if (positionControlled) {
        //     elevatorMotor.setControl(new MotionMagicVoltage(goalState.pos));
        // }
        SmartDashboard.putString("Elevator/goalState", goalState.toString());
        SmartDashboard.putNumber("Elevator/targetPos", goalState.pos);
        SmartDashboard.putNumber("Elevator/measuredPos", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/velocity", elevatorMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Elevator/at goal", atGoal());
    }

    public void moveTo(PositionState position) {
        goalState = position;
    }

    public void setSpeed(double speed) {
        if (speed >= 0) {
            finalSpeed = speed * 0.7;
        } else {
            finalSpeed = speed * 0.3;
        }
        elevatorMotor.set(finalSpeed);
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
