package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command{
    private final ClimberSubsystem climber;
    private Double climberSpeed;

    public ClimberCommand(ClimberSubsystem climber, double climberSpeed){
        this.climber = climber;
        this.climberSpeed = climberSpeed;

        addRequirements(climber);
    }

    @Override
    public void execute(){

        climber.setSpeed(climberSpeed);

    }
    
}
