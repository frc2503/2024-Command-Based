package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.IntakeState;

public class AutoZeroCommand extends Command {

    private EndEffectorSubsystem endEffector;
    private ElevatorSubsystem elevator;

    public AutoZeroCommand(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {
        this.elevator = elevator;
        this.endEffector = endEffector;
        this.endEffector.setIntakeState(IntakeState.EMPTY);
    }

    @Override
    public void execute() {
        if (!elevator.isZeroedForAutonomous()) {
            elevator.zeroPID();
        }
    }

    @Override
    public boolean isFinished() {
        if (elevator.isZeroedForAutonomous()) {
            System.out.println("Auto zeroed");
        }
        return elevator.isZeroedForAutonomous();
    }

    @Override
    public void end(boolean isInterrupted) {
        elevator.setZeroedForAutonomous(true);
        super.end(isInterrupted);
    }
    
}
