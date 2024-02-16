package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command{
    private final ClimberSubsystem climber;
    private DoubleSupplier climberSpeed;

    public ClimberCommand(ClimberSubsystem climber, DoubleSupplier climberSpeed){
        this.climber = climber;
        this.climberSpeed = climberSpeed;

        addRequirements(climber);
    }

    @Override
    public void execute(){

        climber.setSpeed(climberSpeed.getAsDouble());

    }
    
}
