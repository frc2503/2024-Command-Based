package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.EndEffectorSubsystem.IntakeState;

public class AutoStartShootCommand extends Command {

    private EndEffectorSubsystem endEffector;
    private ElevatorSubsystem elevator;

    public AutoStartShootCommand(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        this.endEffector = endEffector;
        this.elevator = elevator;
    }

    @Override
    public void execute() {
        if (elevator.getElevatorState() != ElevatorStates.SHOOT) {
            elevator.shoot();
        } else if (endEffector.getIntakeState() != IntakeState.READY_TO_SHOOT) {
            endEffector.intakeIn();
        }

        elevator.updateElevatorState();
    }

    @Override
    public boolean isFinished() {
        if (elevator.getElevatorState() == ElevatorStates.SHOOT && endEffector.getIntakeState() == IntakeState.READY_TO_SHOOT) {
            System.out.println("Auto ready to shoot");
        }
        return elevator.getElevatorState() == ElevatorStates.SHOOT && endEffector.getIntakeState() == IntakeState.READY_TO_SHOOT;
    }

    @Override
    public void end(boolean isInterrupted) {
        elevator.setZeroedForAutonomous(true);
        super.end(isInterrupted);
    }
    
}
