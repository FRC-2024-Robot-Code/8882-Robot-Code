package frc.robot.commands.Auto.Angles.SourceAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;

public class sAngle1 extends Command {
     Angle robot;
     PIDController pidController;
     double setpoint;

     public sAngle1(Angle robot) {

          this.robot = robot;
          pidController = new PIDController(2, 0, 0);

          addRequirements(robot);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
          double outPut = pidController.calculate(robot.getABSgyro(), 0.171);
          // 0.165
          outPut = MathUtil.clamp(outPut, -0.4, 0.4);

          robot.setAngleSpeed(outPut);
     }

     @Override
     public void end(boolean interrupted) {
          robot.stopAngle();
          this.cancel();
     }

     @Override
     public boolean isFinished() {
          if (pidController.atSetpoint()) {
               return true;
          }
          return false;

     }
}
