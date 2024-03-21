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
    private final double speedMod = -.75;
    private double speed = 0.0;
    private ClimberStates climberState = ClimberStates.LIMBO;

    public ClimberSubsystem(){
        climber = new CANSparkMax(Constants.CLIMBER, MotorType.kBrushless);
        encoder = climber.getEncoder();
        encoder.setPositionConversionFactor(.07331);
        encoder.setPosition(0);
    }

    @Override
    public void periodic(){
        //climber.set(speed*speedMod);
        encoderPosition = -encoder.getPosition();

        if(encoderPosition >= 12.2615327835083){
            climberState = ClimberStates.TOP;
        }else if(encoderPosition <= .2){
            climberState = ClimberStates.BOTTOM;
        }else {
            climberState = ClimberStates.LIMBO;
        }

        if(climberState == ClimberStates.LIMBO){
            climber.set(speed*speedMod);
        }else if(climberState == ClimberStates.TOP && speed < 0){
            climber.set(speed*speedMod);
        }else if(climberState == ClimberStates.BOTTOM && speed > 0){
            climber.set(speed*speedMod);
        }else {
            climber.set(0);
        }

        //System.out.println(encoderPosition);

    }


    public void setSpeed(double speed){
        this.speed = speed;
    }

    public void zeroClimber(){
        encoder.setPosition(0);
    }

    public enum ClimberStates {
        TOP,
        BOTTOM,
        LIMBO
    }

}
