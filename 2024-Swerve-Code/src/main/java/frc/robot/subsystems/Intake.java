package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
     CANSparkMax frontIntake, backIntake;
     String intakeState = "NONE";

     StringLogEntry stateLogEntry = new StringLogEntry(DataLogManager.getLog(), "intake/state");

     public Intake() {
          frontIntake = new CANSparkMax(9, MotorType.kBrushless);
          backIntake = new CANSparkMax(10, MotorType.kBrushless);

          backIntake.setInverted(false);
          frontIntake.setInverted(false);
     }

     public void collectIntake() {
          setIntakeState("INTAKING");
          frontIntake.set(0.9);
          backIntake.set(-0.9);
     }

     public void invertIntake() {
          setIntakeState("INVERTING");
          frontIntake.set(-0.9);
          backIntake.set(-0.9);
     }

     public void stopIntake() {
          setIntakeState("STOPPED");
          frontIntake.stopMotor();
          backIntake.stopMotor();
     }

     public void setIntakeState(String state) {
          this.intakeState = state;
          stateLogEntry.append(state);
     }

     @Override
     public void periodic() {
          SmartDashboard.putString("State Intake", intakeState);

     }
}
