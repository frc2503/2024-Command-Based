package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorCommand extends Command{
    private final EndEffectorSubsystem endEffector;

    public EndEffectorCommand(EndEffectorSubsystem endEffector){
        this.endEffector = endEffector;

        addRequirements(endEffector);
    }
}
