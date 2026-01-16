package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class FieldRelativeDriveCommand extends Command {

  DrivetrainSubsystem drivetrainSubsystem;

  Supplier<Double> xSpeedSupplier, ySpeedSupplier, rSpeedSupplier;

  double xSpeed, ySpeed, rSpeed;


  public FieldRelativeDriveCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    Supplier<Double> xSpeedSupplier,
    Supplier<Double> ySpeedSupplier,
    Supplier<Double> rSpeedSupplier
  ) {
    this.drivetrainSubsystem = drivetrainSubsystem;

    this.xSpeedSupplier = xSpeedSupplier;
    this.ySpeedSupplier = ySpeedSupplier;
    this.rSpeedSupplier = rSpeedSupplier;
    addRequirements(drivetrainSubsystem);

  }

  @Override
  public void initialize() {
    xSpeed = 0;
    ySpeed = 0;
    rSpeed = 0;
  }

  @Override
  public void execute() {

    xSpeed = xSpeedSupplier.get() * -1;
    ySpeed = ySpeedSupplier.get() * -1;
    rSpeed = rSpeedSupplier.get() * -1;

    drivetrainSubsystem.driveFieldRelative(xSpeed, ySpeed, rSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    xSpeed = 0;
    ySpeed = 0;
    rSpeed = 0;
    
    drivetrainSubsystem.driveFieldRelative(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
