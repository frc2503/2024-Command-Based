// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

    public static XboxController driveController = new XboxController(0);
    public static XboxController mechController = new XboxController(1);

    public RobotContainer() {
        drivetrain.register();
        elevator.register();
        endEffector.register();
        climber.register();

        drivetrain.setDefaultCommand(new DriveCommand(
                drivetrain,
                () -> -modifyAxis(driveController.getLeftY()), // Axes are flipped here on purpose
                () -> -modifyAxis(driveController.getLeftX()),
                () -> modifyAxis(driveController.getRightX())
        ));

        climber.setDefaultCommand(new ClimberCommand(climber, modifyAxis(mechController.getLeftY())));

        new Trigger(driveController::getAButton).onTrue(Commands.runOnce(() -> drivetrain.zeroGyroscope(), drivetrain));

        new Trigger(mechController::getLeftBumper).onTrue(Commands.run(() -> endEffector.intakeIn(), endEffector)).onFalse(Commands.runOnce(() -> endEffector.intakeStop(), endEffector));
        new Trigger(mechController::getXButton).onTrue(Commands.run(() -> endEffector.intakeOut(), endEffector)).onFalse(Commands.run(() -> endEffector.intakeStop(), endEffector));
        new Trigger(mechController::getRightBumper).onTrue(Commands.runOnce(() -> endEffector.fire(), endEffector));

        new Trigger(mechController::getAButton).onTrue(Commands.runOnce(() -> elevator.grab(), elevator));
        new Trigger(mechController::getBButton).onTrue(Commands.runOnce(() -> elevator.shoot(), elevator));
        new Trigger(mechController::getYButton).onTrue(Commands.runOnce(() -> elevator.amp(), elevator));

    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
