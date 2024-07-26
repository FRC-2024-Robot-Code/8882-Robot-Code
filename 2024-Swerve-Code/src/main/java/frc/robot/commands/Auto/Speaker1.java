package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class Speaker1 extends SequentialCommandGroup {
     public Speaker1(Shooter shooter, double shooterSpeeed, double conveyorSpeed, double timer1,
               double timer2) {
          addCommands(
                    Commands.runOnce(() -> shooter.setShooterSpeed(shooterSpeeed), shooter),
                    new WaitCommand(timer1),
                    Commands.runOnce(() -> shooter.setConveyorSpeed(conveyorSpeed), shooter),
                    new WaitCommand(timer2),
                    Commands.runOnce(() -> {
                         shooter.stopShooter();
                         shooter.stopConveyor();
                    }, shooter),
                    Commands.runOnce(() -> this.cancel()));
     }
}
