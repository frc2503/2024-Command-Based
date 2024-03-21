package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffectorSubsystem extends SubsystemBase{
    private final CANSparkMax shooterLeft;
    private final CANSparkMax shooterRight;
    private final CANSparkMax intakeBottom;
    private RelativeEncoder shooterSpeedEncoder;
    private final DigitalInput noteSensor;
    private IntakeState intakeState = IntakeState.EMPTY;
    private ShooterStates shooterState = ShooterStates.IDLE;

    public EndEffectorSubsystem(){
        shooterLeft = new CANSparkMax(Constants.SHOOTER_LEFT, MotorType.kBrushless);
        shooterRight = new CANSparkMax(Constants.SHOOTER_RIGHT, MotorType.kBrushless);
        intakeBottom = new CANSparkMax(Constants.INTAKE_BOTTOM, MotorType.kBrushless);
        noteSensor = new DigitalInput(1);
        shooterSpeedEncoder = shooterRight.getEncoder();
        shooterSpeedEncoder.setVelocityConversionFactor(1);

        shooterLeft.setOpenLoopRampRate(.5);
        shooterRight.setOpenLoopRampRate(.5);
    }

    @Override
    public void periodic(){
       SmartDashboard.putBoolean("Has Note", intakeState == IntakeState.HAS_NOTE);
    }

    public void intakeIn(){

        if(intakeState == IntakeState.EMPTY){
            intakeBottom.set(-.60);
            if(noteSensor.get() == false){
                intakeState = IntakeState.HAS_NOTE;
            }
        }else if(intakeState == IntakeState.HAS_NOTE){
            intakeBottom.set(.05);
            if(noteSensor.get() == true){
                intakeState = IntakeState.READY_TO_SHOOT;
            }
        }else if(intakeState == IntakeState.READY_TO_SHOOT){
            intakeStop();
        }

        //System.out.println("Beam broken: " + !noteSensor.get());
        //System.out.println("Intake state: " + intakeState.name());
        
    }

    public void intakeOut(){
        intakeBottom.set(.35);
        intakeState = IntakeState.EMPTY;
    }

    public void intakeStop(){
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
        intakeBottom.set(-.15);
    }

    private void ampRamp(){
        shooterLeft.set(.35);
        shooterRight.set(-.35);
    }

    public void fire(){
        if(ElevatorSubsystem.elevatorSetpoint != "amp"){
            if(intakeState == IntakeState.READY_TO_SHOOT && shooterState == ShooterStates.IDLE){
                    rampUp();
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
                    intakeStop();
                    rampDown();
                }
            }else{
                intakeStop();
                rampDown();
            }
        }else if(ElevatorSubsystem.elevatorSetpoint == "amp"){
            if(intakeState == IntakeState.READY_TO_SHOOT && shooterState == ShooterStates.IDLE){
                ampRamp();
                if(shooterSpeedEncoder.getVelocity() <= -1985){
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
                    intakeStop();
                    rampDown();
                }
            }else{
                intakeStop();
                rampDown();
            }
        }
    }

    public void setIntakeState(IntakeState state) {
        this.intakeState = state;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public enum IntakeState {
        EMPTY,
        HAS_NOTE,
        READY_TO_SHOOT
    }

    public enum ShooterStates {
        IDLE,
        RAMPED_UP,
        FIRING
    }
    
}
