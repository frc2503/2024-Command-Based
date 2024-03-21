package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.IntakeState;

public class AutoShootCommand extends Command {
    
    private EndEffectorSubsystem endEffector;

    public AutoShootCommand(EndEffectorSubsystem endEffector){
        this.endEffector = endEffector;
    }

    @Override
    public void execute() {
        if(endEffector.getIntakeState() == IntakeState.READY_TO_SHOOT){
            endEffector.fire();
        }
    }

    @Override
    public boolean isFinished() {
        return endEffector.getIntakeState() == IntakeState.EMPTY;
    }
}
