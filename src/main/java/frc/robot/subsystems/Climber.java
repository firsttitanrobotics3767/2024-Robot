package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Climber extends SubsystemBase{
    private static Climber instance = null;
    private double targetOpenLoopOutput = 0;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }
    
    private final CANSparkMax leader;
    private final CANSparkMax follower;

    // private final AbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder encoder;
    private final DigitalInput limitSwitch;

    // private final TalonFX climbMotor;
    // private final TalonFXConfiguration config;


    public Climber() {
        // climbMotor = new TalonFX(21);
        // config = new TalonFXConfiguration();
        // climbMotor.setNeutralMode(NeutralModeValue.Brake);
        // climbMotor.setInverted(false);
        // climbMotor.getConfigurator().apply(config);
        leader = new CANSparkMax(Constants.Climber.leaderID, MotorType.kBrushless);
        leader.restoreFactoryDefaults();
        leader.setIdleMode(IdleMode.kBrake);
        leader.setSmartCurrentLimit(100);
        leader.setInverted(false);

        follower = new CANSparkMax(Constants.Climber.followerID, MotorType.kBrushless);
        follower.restoreFactoryDefaults();
        follower.setIdleMode(IdleMode.kBrake);
        follower.setSmartCurrentLimit(100);
        follower.setInverted(false);
        follower.follow(leader);

        // absoluteEncoder = leader.getAbsoluteEncoder(Type.kDutyCycle);

        encoder = leader.getEncoder();
        encoder.setPositionConversionFactor(Constants.Climber.conversionfactor);

        limitSwitch = new DigitalInput(4);
        // encoder.setPosition(absoluteEncoder.getPosition());


    }

    @Override
    public void periodic() {
        if (DriverStation.getMatchTime() <= 20) {
            if (!limitSwitch.get()) {

                if (targetOpenLoopOutput > 0) {
                    leader.set(targetOpenLoopOutput * 0.5);
                } else{
                    leader.set(0);
                }
                
            } else if (encoder.getPosition() >= 38 && targetOpenLoopOutput > 0) {
                leader.set(0);
            } else {
                leader.set(targetOpenLoopOutput * 0.5);
            }
        } else {
            leader.set(0);
        }

        // leader.set(targetOpenLoopOutput);

        SmartDashboard.putNumber("Absolute Climber", encoder.getPosition());
        SmartDashboard.putBoolean("Climber limit", limitSwitch.get());
        // SmartDashboard.putNumber("climber output", targetOpenLoopOutput);
        // // climbMotor.set(targetOpenLoopOutput);
    } 

    public void setArmSpeed(double speed) {
        targetOpenLoopOutput = speed;
    }

    // public double getPosition() {
    //     return absoluteEncoder.getPosition();
    // }

}
