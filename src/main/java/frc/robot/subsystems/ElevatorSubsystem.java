package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private final CANSparkMax elevator;
    private final SparkPIDController elevatorPID;
    private final RelativeEncoder encoder;
    public static String elevatorSetpoint = "amp";
    
    public ElevatorSubsystem(){
        elevator = new CANSparkMax(Constants.ELEVATOR, MotorType.kBrushless);
        elevatorPID = elevator.getPIDController();
        encoder = elevator.getEncoder();

        encoder.setPositionConversionFactor(.0052338);

        elevatorPID.setFeedbackDevice(encoder);
        setPIDGains(0, 1, 0, 0);
        elevatorPID.setOutputRange(-1, 1);
        elevatorPID.setSmartMotionMaxVelocity(.66, 0);
        elevatorPID.setSmartMotionMaxAccel(.44, 0);
    }

    public void setPIDGains(double FF, double P, double I, double D){
        elevatorPID.setFF(FF);
        elevatorPID.setP(P);
        elevatorPID.setI(I);
        elevatorPID.setD(D);
    }

    @Override
    public void periodic(){
        
    }

    public void grab(){
        elevatorPID.setReference(Units.inchesToMeters(18.25*2+7), ControlType.kPosition);
        elevatorSetpoint = "grab";
    }

    public void shoot(){
        elevatorPID.setReference(Units.inchesToMeters(12), ControlType.kPosition);
        elevatorSetpoint = "shoot";
    }

    public void amp(){
        elevatorPID.setReference(0, ControlType.kPosition);
        elevatorSetpoint = "amp";
    }

}
