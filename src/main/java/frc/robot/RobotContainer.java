// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autonomous.AutoPickUpCommand;
import frc.robot.commands.autonomous.BasicAutonomousCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.IntakeState;
import frc.robot.vision.AprilTagIds;
import frc.robot.vision.AprilTagLock;
import frc.robot.vision.LimelightHelpers;
import edu.wpi.first.cameraserver.CameraServer;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    public static XboxController driveController = new XboxController(1);
    public static XboxController mechController = new XboxController(0);

    //private final SendableChooser<Command> autoChooser;

    private AprilTagLock aprilTagLock = new AprilTagLock();

    public RobotContainer() {
        drivetrain.register();
        elevator.register();
        endEffector.register();
        climber.register();

        //NamedCommands.registerCommand("AutoPickUpCommand", new AutoPickUpCommand(elevator, endEffector));

        new Trigger(driveController::getAButton).onTrue(Commands.runOnce(() -> drivetrain.zeroGyroscope(), drivetrain));

        new Trigger(mechController::getLeftBumper).onTrue(Commands.run(() -> endEffector.intakeIn(), endEffector)).onFalse(Commands.runOnce(() -> endEffector.intakeStop(), endEffector));
        new Trigger(mechController::getXButton).onTrue(Commands.run(() -> endEffector.intakeOut(), endEffector)).onFalse(Commands.runOnce(() -> endEffector.intakeStop(), endEffector));
        new Trigger(mechController::getRightBumper).onTrue(Commands.run(() -> endEffector.fire(), endEffector)).onFalse(Commands.runOnce(() -> endEffector.rampDown(), endEffector));

        new Trigger(mechController::getAButton).onTrue(Commands.runOnce(() -> elevator.grab(), elevator));
        new Trigger(mechController::getBButton).onTrue(Commands.runOnce(() -> elevator.shoot(), elevator));
        new Trigger(mechController::getYButton).onTrue(Commands.runOnce(() -> elevator.amp(), elevator)).onFalse(Commands.runOnce(() -> elevator.shoot(), elevator));
        new Trigger(mechController::getStartButton).onTrue(Commands.run(() -> elevator.zeroPID(), elevator)).onFalse(Commands.runOnce(() -> elevator.stopElevator(), elevator));
        //new Trigger(mechController::getBackButton).onTrue(Commands.run(() -> , climber))

        //autoChooser = AutoBuilder.buildAutoChooser();
        //SmartDashboard.putData("Auto Chooser", autoChooser);

        //CameraServer.startAutomaticCapture();

    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    public void onTeleopInit() {
        drivetrain.setDefaultCommand(new DriveCommand(
                drivetrain,
                () -> -speedModify(driveController.getLeftY()), // Axes are flipped here on purpose
                () -> -speedModify(driveController.getLeftX()),
                () -> speedModify(getDriveRotation())
        ));

        climber.setDefaultCommand(new ClimberCommand(climber, () -> -climbModify(mechController.getLeftY())));
        Commands.runOnce(() -> elevator.shoot(), elevator);
        endEffector.setIntakeState(IntakeState.EMPTY);
    }

    private double getDriveRotation() {
        double controllerAngle = modifyAxis(driveController.getRightX());
        if (driveController.getRightBumperPressed()) {
            // Holding right bumper of drive controller will try to lock onto AprilTag above speaker
            Double aprilTagAngle = aprilTagLock.getAngleToTag(AprilTagIds.getSpeakerTagId());
            if (aprilTagAngle != null) {
                return aprilTagAngle;
            } else {
                return controllerAngle;
            }
        } else {
            return controllerAngle;
        }
    }

    public Command getAutoCommand() {
        return new BasicAutonomousCommand(drivetrain, elevator, endEffector);
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
        value = Math.copySign(value * value, value) * 0.75;

        return value;
    }

    private static double speedModify(double value) {
        if(driveController.getLeftBumper()){
            value = .2*modifyAxis(value);
        }else{
           value = modifyAxis(value);
        }

        return value;

    }

    private static double climbModify(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value) * 0.75;

        return value;
    }

    // public Command getAutonomousCommand() {
    //     return autoChooser.getSelected();
    // }
}
