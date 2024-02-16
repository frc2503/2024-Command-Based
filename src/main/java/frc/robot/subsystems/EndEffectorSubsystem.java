package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class EndEffectorSubsystem extends SubsystemBase{
    private final CANSparkMax shooterLeft;
    private final CANSparkMax shooterRight;
    private final CANSparkMax intakeTop;
    private final CANSparkMax intakeBottom;
    private final DigitalInput noteSensor;
    public Boolean hasNote = false;
    public Boolean hasRumbled = false;
    public Boolean reversed = false;
    private static XboxController driveController;
    private static XboxController mechController;

    public EndEffectorSubsystem(){
        shooterLeft = new CANSparkMax(Constants.SHOOTER_LEFT, MotorType.kBrushless);
        shooterRight = new CANSparkMax(Constants.SHOOTER_RIGHT, MotorType.kBrushless);
        intakeTop = new CANSparkMax(Constants.INTAKE_TOP, MotorType.kBrushless);
        intakeBottom = new CANSparkMax(Constants.INTAKE_BOTTOM, MotorType.kBrushless);
        noteSensor = new DigitalInput(0);
        driveController = RobotContainer.driveController;
        mechController = RobotContainer.mechController;
    }

    @Override
    public void periodic(){
        if(noteSensor.get() == true){
            hasNote = true;
        }else{
            hasNote = false;
            hasRumbled = false;
        }

        /*if(hasNote == true && hasRumbled == false){
            rumble();
        }*/
    }

    public void intakeIn(){
        if(noteSensor.get() == false && hasNote == false){
            gotNote();
            hasNote = true;
        }else if(noteSensor.get() == true){
            hasNote = false;
            hasRumbled = false;
        }

        if(hasNote == false){
        intakeTop.set(.25);
        intakeBottom.set(.25);
        }
        
    }

    public void intakeOut(){
        intakeTop.set(-.15);
        intakeBottom.set(-.15);
    }

    public void intakeStop(){
        intakeTop.set(0);
        intakeBottom.set(0);
    }

    public void rumble(){
        driveController.setRumble(RumbleType.kBothRumble, .75);
        mechController.setRumble(RumbleType.kBothRumble, .75);
        Timer.delay(2);
        driveController.setRumble(RumbleType.kBothRumble, 0);
        mechController.setRumble(RumbleType.kBothRumble, 0);
        hasRumbled = true;
    }

    private void rampUp(){
        shooterLeft.set(.90);
        shooterRight.set(-.95);
    }

    private void rampDown(){
        shooterLeft.set(0);
        shooterRight.set(0);
    }

    private void intakeFire(){
        intakeTop.set(.15);
        intakeBottom.set(.15);
    }

    public void fire(){
        if(ElevatorSubsystem.elevatorSetpoint == "shoot"){
            rampUp();
            Timer.delay(1);
            intakeFire();
            Timer.delay(1);
            rampDown();
            intakeStop();
        }else if(ElevatorSubsystem.elevatorSetpoint == "amp"){
            intakeOut();
            Timer.delay(3);
            intakeStop();
        }
    }

    public void gotNote(){
        intakeTop.set(-.05);
        intakeBottom.set(-.05);
        Timer.delay(.001);
        intakeStop();
        reversed = true;
    }
    
}
