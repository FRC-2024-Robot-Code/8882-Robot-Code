package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCmd extends Command {

  Shooter shooter;

  public ShooterCmd(Shooter subsystem) {
    shooter = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // shooter.setSpeedConveyor(0.25);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    shooter.stopMotorConveyor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}