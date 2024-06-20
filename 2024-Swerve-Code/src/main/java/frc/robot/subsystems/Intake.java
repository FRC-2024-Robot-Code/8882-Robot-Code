package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  CANSparkMax frontIntake, backIntake;

  String state = "NONE";

  StringLogEntry stateLogEntry = new StringLogEntry(DataLogManager.getLog(), "intake/states");

  public Intake() {
    frontIntake = new CANSparkMax(9, MotorType.kBrushless);
    backIntake = new CANSparkMax(10, MotorType.kBrushless);

    backIntake.setInverted(true);
  }

  public void collect() {
    setState("INTAKING");
    frontIntake.set(0.8);
    backIntake.set(0.9);
  }

  public void collectAuto() {
    frontIntake.set(0.8);
    backIntake.set(0.9);
  }

  public void invert() {
    setState("INVERTING");
    frontIntake.set(-0.8);
    backIntake.set(-0.8);
  }

  public void stop() {
    setState("STOPPED");
    frontIntake.stopMotor();
    backIntake.stopMotor();
  }

  public void setState(String state) {
    this.state = state;
    stateLogEntry.append(state);
  }

  @Override
  public void periodic() {
  }
}
