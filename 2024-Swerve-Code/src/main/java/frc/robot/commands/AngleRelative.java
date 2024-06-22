package frc.robot.commands;

import javax.xml.crypto.Data;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;

public class AngleRelative extends Command {

     Angle robot;
     PIDController pidController;

     double setpoint;

     DoubleLogEntry setpointLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "angle/setpoint");
     DoubleLogEntry outputLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "angle/velocity");

     public AngleRelative(Angle robot, double setpoint) {
          this.robot = robot;
          this.setpoint = setpoint;
          pidController = new PIDController(0.5, 0, 0);
          pidController.setSetpoint(setpoint);
          setpointLogEntry.append(setpoint);

          addRequirements(robot);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
          double outPut = pidController.calculate(robot.getAnglePosition());

          outPut = MathUtil.clamp(outPut, -0.2, 0.2);

          robot.setAngleSpeed(outPut);
          outputLogEntry.append(outPut);
     }

     @Override
     public void end(boolean interrupted) {
          robot.stopAngle();
     }

     @Override
     public boolean isFinished() {
          return false;
     }
}
