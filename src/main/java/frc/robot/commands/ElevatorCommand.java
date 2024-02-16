package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command{
    private final ElevatorSubsystem elevator;

    public ElevatorCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
    }
    
}
