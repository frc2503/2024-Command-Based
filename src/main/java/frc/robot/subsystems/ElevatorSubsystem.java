package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

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
    public static String elevatorSetpoint;
    private DigitalInput zeroSwitch;
    private boolean isZeroedForAutonomous = false;
    public ElevatorStates elevatorState = ElevatorStates.AMP;
    
    public ElevatorSubsystem(){
        elevator = new CANSparkMax(Constants.ELEVATOR, MotorType.kBrushless);
        elevatorPID = elevator.getPIDController();
        encoder = elevator.getEncoder();

        zeroSwitch = new DigitalInput(0);

        encoder.setPositionConversionFactor(.00392535);

        elevatorPID.setFeedbackDevice(encoder);
        setPIDGains(0, 5.5, 0, 0);
        elevatorPID.setOutputRange(-1, 1);
        elevatorPID.setSmartMotionMaxVelocity(2.5, 0);
        elevatorPID.setSmartMotionMaxAccel(1, 0);
    }

    public void setPIDGains(double FF, double P, double I, double D){
        elevatorPID.setFF(FF);
        elevatorPID.setP(P);
        elevatorPID.setI(I);
        elevatorPID.setD(D);
    }

    @Override
    public void periodic(){
       // System.out.println(zeroSwitch.get());
        //System.out.println("Elevator position: " + encoder.getPosition());
        if(elevatorSetpoint == "grab" && encoder.getPosition() <= Units.inchesToMeters(0) && encoder.getPosition() >= -Units.inchesToMeters(.5)){
            elevatorState = ElevatorStates.GRAB;
        }else if(elevatorSetpoint == "shoot" && encoder.getPosition() <= -Units.inchesToMeters(28) && encoder.getPosition() >= -Units.inchesToMeters(29)){
            elevatorState = ElevatorStates.SHOOT;
        }else if(elevatorSetpoint == "amp" && encoder.getPosition() <= -Units.inchesToMeters(42.5) && encoder.getPosition() >= -Units.inchesToMeters(43.5)){
            elevatorState = ElevatorStates.AMP;
        }else {
            elevatorState = ElevatorStates.MOVING;
        }
        
    }

    public void updateElevatorState() {
        
    }

    public void grab(){
        elevatorPID.setReference(Units.inchesToMeters(0), ControlType.kPosition);
        elevatorSetpoint = "grab";
    }

    public void shoot(){
        elevatorPID.setReference(-Units.inchesToMeters(28.5), ControlType.kPosition);
        elevatorSetpoint = "shoot";
    }

    // public void shootPlus(){
    //     elevatorPID.setReference(27.5, ControlType.kPosition);
    //     elevatorSetpoint = "shoot";
    // }

    public void amp(){
        elevatorPID.setReference(-Units.inchesToMeters(44), ControlType.kPosition);
        elevatorSetpoint = "amp";
    }

    public void zeroPID(){
        if(zeroSwitch.get() == true){
            elevator.set(.5);
        }else{
            isZeroedForAutonomous = true;
            elevator.set(0);
            encoder.setPosition(0);
            elevatorSetpoint = "grab";
        }
        
    }

    public void stopElevator(){
        elevator.set(0);
    }

    public void manualElevatorControl(DoubleSupplier speed){
        elevator.set(speed.getAsDouble()*-.2);
    }

    public boolean isZeroedForAutonomous() {
        return isZeroedForAutonomous;
    }

    public void setZeroedForAutonomous(boolean isZeroed) {
        this.isZeroedForAutonomous = isZeroed;
    }

    public ElevatorStates getElevatorState() {
        return elevatorState;
    }

    public void setElevatorState(ElevatorStates state) {
        this.elevatorState = state;
    }

    public enum ElevatorStates{
        AMP,
        SHOOT,
        GRAB,
        MOVING
    }

}
