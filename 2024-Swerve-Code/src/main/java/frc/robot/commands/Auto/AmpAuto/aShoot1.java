package frc.robot.commands.Auto.AmpAuto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class aShoot1 extends SequentialCommandGroup {
     public aShoot1(Shooter shooter) {
          addCommands(
                    Commands.runOnce(() -> shooter.setSpeed(0.65), shooter),
                    new WaitCommand(1.5),
                    Commands.runOnce(() -> shooter.setSpeedConveyor(1), shooter),
                    new WaitCommand(1.5),
                    Commands.runOnce(() -> shooter.stopAll(), shooter),
                    Commands.runOnce(() -> this.cancel()));
     }
}
