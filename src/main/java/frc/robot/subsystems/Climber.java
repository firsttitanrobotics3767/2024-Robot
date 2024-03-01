package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Climber extends SubsystemBase{
    private static Climber instance = null;
    private boolean openLoopControl = Constants.defaultControlMode;
    private double targetOpenLoopOutput = 0;
    // private double targetPos = Superstructure.ClimberState.STOW.pos;
    private double targetPos = 0;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }
    
    private final CANSparkMax leader;
    private final CANSparkMax follower;

    private final AbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder encoder;

    private final SparkPIDController pidController;

    public Climber() {
        leader = new CANSparkMax(Constants.Climber.leaderID, MotorType.kBrushless);
        leader.restoreFactoryDefaults();
        leader.setIdleMode(IdleMode.kBrake);
        leader.setInverted(false);

        follower = new CANSparkMax(Constants.Climber.followerID, MotorType.kBrushless);
        follower.restoreFactoryDefaults();
        follower.setIdleMode(IdleMode.kBrake);
        follower.setInverted(false);
        follower.follow(leader);

        absoluteEncoder = leader.getAbsoluteEncoder(Type.kDutyCycle);

        encoder = leader.getEncoder();
        encoder.setPositionConversionFactor(Constants.Climber.conversionfactor);
        encoder.setPosition(absoluteEncoder.getPosition());

        pidController = leader.getPIDController();
        pidController.setFeedbackDevice(absoluteEncoder);
        pidController.setP(Constants.Climber.kP);
        pidController.setI(Constants.Climber.kI);
        pidController.setD(Constants.Climber.kD);
        pidController.setSmartMotionMaxAccel(Constants.Climber.maxAccel, 0);
        pidController.setSmartMotionMaxVelocity(Constants.Climber.maxVel, 0);

    }

    @Override
    public void periodic() {
            leader.set(targetOpenLoopOutput);
    } 

    public void moveTo(double position) {
        targetPos = position;
    }

    public void setOpenLoopControl(boolean controlType) {
        openLoopControl = controlType;
    }

    public void setArmSpeed(double speed) {
        targetOpenLoopOutput = speed;
    }

    public double getPosition() {
        return absoluteEncoder.getPosition();
    }

}
