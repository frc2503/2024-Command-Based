package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
    private final CANSparkMax climber;
    private final RelativeEncoder encoder;
    private double encoderPosition;
    private final double speedMod = .5;
    private double speed = 0.0;

    public ClimberSubsystem(){
        climber = new CANSparkMax(Constants.CLIMBER, MotorType.kBrushless);
        encoder = climber.getEncoder();
        encoder.setPositionConversionFactor(.00079331);
        encoder.setPosition(0);
    }

    @Override
    public void periodic(){
        // if(speed > 0 && encoderPosition < Units.inchesToMeters(15.075)){
        //     climberOut(speed);
        // }else if(speed < 0 && encoderPosition > 0){
        //     climberIn(speed);
        // }else{
        //     climber.set(0);
        // }
        System.out.println("Speed = " + speed);
        System.out.println("Encoder = " + encoderPosition);
        climber.set(speed);
        encoderPosition = -encoder.getPosition();
    }

    private void climberOut(double speed){
        climber.set(speed);
    }

    private void climberIn(double speed){
        climber.set(speed);
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

}
