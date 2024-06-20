package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.Shooter;

public class ShootSource extends SequentialCommandGroup {

  public ShootSource(Shooter shooter, AngleShooter angle) {
    addCommands(
        Commands.runOnce(() -> angle.setTarget(0.73), angle),
        Commands.runOnce(() -> shooter.setSpeed(0.30), shooter),
        new WaitCommand(1.5),
      Commands.runOnce(() -> shooter.setSpeedConveyor(1), shooter),
      new WaitCommand(2),
      Commands.runOnce(
        () -> {
          shooter.stopMotor();
          shooter.stopMotorConveyor();
        },
        shooter
      ),
      Commands.runOnce(() -> this.cancel())
    );
  }
}