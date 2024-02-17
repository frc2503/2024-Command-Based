package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private final CANSparkMax elevator;
    private final SparkPIDController elevatorPID;
    private final RelativeEncoder encoder;
    public static String elevatorSetpoint = "amp";
    private DigitalInput zeroSwitch;
    
    public ElevatorSubsystem(){
        elevator = new CANSparkMax(Constants.ELEVATOR, MotorType.kBrushless);
        elevatorPID = elevator.getPIDController();
        encoder = elevator.getEncoder();

        zeroSwitch = new DigitalInput(1);

        encoder.setPositionConversionFactor(.0052338);

        elevatorPID.setFeedbackDevice(encoder);
        setPIDGains(0, 1, 0, 0);
        elevatorPID.setOutputRange(-1, 1);
        elevatorPID.setSmartMotionMaxVelocity(1, 0);
        elevatorPID.setSmartMotionMaxAccel(.66, 0);
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
        elevatorPID.setReference(Units.inchesToMeters(0), ControlType.kPosition);
        elevatorSetpoint = "grab";
    }

    public void shoot(){
        elevatorPID.setReference(-Units.inchesToMeters(31.5), ControlType.kPosition);
        elevatorSetpoint = "shoot";
    }

    public void amp(){
        elevatorPID.setReference(-Units.inchesToMeters(18.25*2+7), ControlType.kPosition);
        elevatorSetpoint = "amp";
    }

    public void zeroPID(){
        if(zeroSwitch.get() == true){
            elevator.set(.3);
        }else{
            elevator.set(0);
            encoder.setPosition(0);
        }
        
    }

    public void stopElevator(){
        elevator.set(0);
    }

}
