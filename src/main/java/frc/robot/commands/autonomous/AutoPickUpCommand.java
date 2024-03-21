package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.EndEffectorSubsystem.IntakeState;

public class AutoPickUpCommand extends Command{

    private ElevatorSubsystem elevator;
    private EndEffectorSubsystem endEffector;

    public AutoPickUpCommand(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {
        this.elevator = elevator;
        this.endEffector = endEffector;
    }

    @Override
    public void execute() {
        if(endEffector.getIntakeState() == IntakeState.EMPTY){
            endEffector.intakeIn();
        }else if(endEffector.getIntakeState() == IntakeState.HAS_NOTE){
            elevator.shoot();
        }
    }

    @Override
    public boolean isFinished() {
        return endEffector.getIntakeState() == IntakeState.READY_TO_SHOOT && elevator.getElevatorState() == ElevatorStates.SHOOT;
    }
    
}
