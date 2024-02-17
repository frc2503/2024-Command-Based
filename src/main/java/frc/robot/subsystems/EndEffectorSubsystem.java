package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class EndEffectorSubsystem extends SubsystemBase{
    private final CANSparkMax shooterLeft;
    private final CANSparkMax shooterRight;
    private final CANSparkMax intakeTop;
    private final CANSparkMax intakeBottom;
    private RelativeEncoder shooterSpeedEncoder;
    private final DigitalInput noteSensor;
    private static XboxController driveController;
    private static XboxController mechController;
    private IntakeState intakeState = IntakeState.EMPTY;
    private ShooterStates shooterState = ShooterStates.IDLE;

    public EndEffectorSubsystem(){
        shooterLeft = new CANSparkMax(Constants.SHOOTER_LEFT, MotorType.kBrushless);
        shooterRight = new CANSparkMax(Constants.SHOOTER_RIGHT, MotorType.kBrushless);
        intakeTop = new CANSparkMax(Constants.INTAKE_TOP, MotorType.kBrushless);
        intakeBottom = new CANSparkMax(Constants.INTAKE_BOTTOM, MotorType.kBrushless);
        noteSensor = new DigitalInput(0);
        driveController = RobotContainer.driveController;
        mechController = RobotContainer.mechController;
        shooterSpeedEncoder = shooterRight.getEncoder();

        shooterLeft.setOpenLoopRampRate(2);
        shooterRight.setOpenLoopRampRate(2);
    }

    @Override
    public void periodic(){
       
    }

    public void intakeIn(){

        if(intakeState == IntakeState.EMPTY){
            intakeTop.set(.25);
            intakeBottom.set(.25);
            if(noteSensor.get() == false){
                intakeState = IntakeState.HAS_NOTE;
            }
        }else if(intakeState == IntakeState.HAS_NOTE){
            intakeTop.set(-.05);
            intakeBottom.set(-.05);
            if(noteSensor.get() == true){
                intakeState = IntakeState.READY_TO_SHOOT;
            }
        }else if(intakeState == IntakeState.READY_TO_SHOOT){
            intakeStop();
        }
        
    }

    public void intakeOut(){
        intakeTop.set(-.15);
        intakeBottom.set(-.15);
        intakeState = IntakeState.EMPTY;
    }

    public void intakeStop(){
        intakeTop.set(0);
        intakeBottom.set(0);
    }


    private void rampUp(){
        shooterLeft.set(1);
        shooterRight.set(-1);
    }

    public void rampDown(){
        shooterLeft.set(0);
        shooterRight.set(0);
    }

    private void intakeFire(){
        intakeTop.set(.15);
        intakeBottom.set(.15);
    }

    public void fire(){
        if(ElevatorSubsystem.elevatorSetpoint == "shoot"){
            if(intakeState == IntakeState.READY_TO_SHOOT && shooterState == ShooterStates.IDLE){
                rampUp();
                System.out.println("Shooter RPM = " + shooterSpeedEncoder.getVelocity());
                if(shooterSpeedEncoder.getVelocity() <= -5600){
                    shooterState = ShooterStates.RAMPED_UP;
                }
            }else if(shooterState == ShooterStates.RAMPED_UP){
                intakeFire();
                if(noteSensor.get() == false){
                    shooterState = ShooterStates.FIRING;
                }
            }else if(shooterState == ShooterStates.FIRING){
                if(noteSensor.get() == true){
                    shooterState = ShooterStates.IDLE;
                    intakeState = IntakeState.EMPTY;
                }
            }else{
                intakeStop();
                rampDown();
            }
        }
    }

    private enum IntakeState {
        EMPTY,
        HAS_NOTE,
        READY_TO_SHOOT
    }

    private enum ShooterStates {
        IDLE,
        RAMPED_UP,
        FIRING
    }
    
}
