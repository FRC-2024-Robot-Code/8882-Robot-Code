package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
     CANSparkMax blackShooter, greeenShooter, conveyor;
     DigitalInput irSensor = new DigitalInput(0);

     String irSensoString = "OFF";
     StringLogEntry irSensorLogEntry = new StringLogEntry(DataLogManager.getLog(), "shooter/ir");

     public Shooter() {
          blackShooter = new CANSparkMax(13, MotorType.kBrushless);
          greeenShooter = new CANSparkMax(14, MotorType.kBrushless);
          conveyor = new CANSparkMax(15, MotorType.kBrushless);

          greeenShooter.setInverted(true);
          blackShooter.setInverted(true);
          conveyor.setInverted(true);
          greeenShooter.follow(blackShooter);
     }

     public boolean getIrState() {
          return irSensor.get();
     }

     public void setShooterSpeed(double speed) {
          blackShooter.set(speed);
     }

     public void setConveyorSpeed(double speed) {
          conveyor.set(speed);
     }

     public void collectConveyor(double speed) {
          if (getIrState()) {
               conveyor.stopMotor();
          } else {
               conveyor.set(speed);
          }
     }

     public void stopShooter() {
          blackShooter.stopMotor();
     }

     public void stopConveyor() {
          conveyor.stopMotor();
     }

     @Override
     public void periodic() {
          SmartDashboard.putBoolean("State IR", getIrState());

     }
}
