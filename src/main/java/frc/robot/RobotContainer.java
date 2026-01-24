// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.FieldRelativeDriveCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class RobotContainer {

  DrivetrainSubsystem drivetrainSubsystem;
  CommandXboxController driverController;

  FieldRelativeDriveCommand fieldRelativeDriveCommand;

  public RobotContainer() {
    drivetrainSubsystem = new DrivetrainSubsystem();
    driverController = new CommandXboxController(0);

    fieldRelativeDriveCommand = new FieldRelativeDriveCommand(
        drivetrainSubsystem,
        driverController::getLeftY,
        driverController::getLeftX,
        driverController::getRightX);

    configureBindings();
  }

  private void configureBindings() {
    drivetrainSubsystem.setDefaultCommand(fieldRelativeDriveCommand);

    // brake with A button.
    // TODO: Learn the SwerveDriveRequest System or an Alternative for YAGSL
    driverController.a().whileTrue(
      Commands.run(() -> drivetrainSubsystem.lockWheels(), drivetrainSubsystem));
      
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
