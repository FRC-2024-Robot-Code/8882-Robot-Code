package frc.robot.commands.Auto.Angles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;

public class AngleForIntake extends Command {
     Angle robot;
     PIDController pidController;

     double setpoint;

     public AngleForIntake(Angle robot) {

          this.robot = robot;
          pidController = new PIDController(3, 0, 0);

          addRequirements(robot);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
          double outPut = pidController.calculate(robot.getABSgyro(), 0.5);

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
          return pidController.atSetpoint();
     }
}
