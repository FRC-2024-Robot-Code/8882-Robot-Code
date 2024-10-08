// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Controle;
import frc.robot.Constants.Tracao;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

/**
 * Classe que calcula a partir da entrada do gamepad a saída do swerve
 */
public class Teleop extends Command {

  // Variáveis que guardam nossas funções do gamepad
  DoubleSupplier y;
  DoubleSupplier x;
  DoubleSupplier turn;

  // Objetos necessárias para acessar funções e variáveis
  Swerve swerve;
  SwerveController controller;

  // Variáveis que guardam a translação e velocidade angular do swerve
  Translation2d translation;
  double angle;
  double omega;

  XboxController controle1;

  public Teleop(
      Swerve swerve,
      DoubleSupplier y,
      DoubleSupplier x,
      DoubleSupplier turn,
      XboxController controle1) {
    // Aqui atribuimos as funções e subsistema
    this.y = y;
    this.x = x;
    this.turn = turn;
    this.swerve = swerve;
    this.controle1 = controle1;

    controller = swerve.getSwerveController(); // Obtemos o controlador do swerve
    // Adiciona a tração como requerimento
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Abaixo calculamos os valores de saída a partir dos nossos inputs
  @Override
  public void execute() {

    double angVelocity;
    double xVelocity;
    double yVelocity;
    xVelocity = y.getAsDouble() * Tracao.multiplicadorTranslacionalY;
    yVelocity = x.getAsDouble() * Tracao.multiplicadorTranslacionalX;
    angVelocity = turn.getAsDouble() * Tracao.multiplicadorRotacional;

    omega = controller.config.maxAngularVelocity * angVelocity;

    if (controle1.getRawAxis(Controle.rightTrigger) != 0) {
      translation = new Translation2d(xVelocity * 0.45, yVelocity * 0.45);
    } else if (controle1.getRawAxis(Controle.leftTrigger) != 0) {
      translation = new Translation2d(xVelocity * 2, yVelocity * 2);
    } else {
      translation = new Translation2d(
          xVelocity * Tracao.MAX_SPEED,
          yVelocity * Tracao.MAX_SPEED);
    }

    // Caso essa função seja verdadeira a aceleração do robô será limitada
    swerve.drive(translation, omega, Tracao.fieldRelative);

    // Aqui temos nossa função definida dentro da classe de subsistema a qual
    // comandara o swerve
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
