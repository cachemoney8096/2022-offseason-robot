// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.cachemoney8096.frc2022o.libs_3005.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import net.cachemoney8096.frc2022o.libs_3005.util.SendableHelper;

public class ProfiledPositionController implements Controller {
  private double m_dt = 0.02;
  private final Controller m_positionController;
  private final TrapezoidProfile.Constraints m_profileConstraints;
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private double m_minimumInput = 0.0;
  private double m_maximumInput = 0.0;

  public ProfiledPositionController(
      Controller positionController, TrapezoidProfile.Constraints constraints, double loopPeriod) {
    m_positionController = positionController;
    m_profileConstraints = constraints;
    m_dt = loopPeriod;
  }

  public ProfiledPositionController(
      Controller positionController, TrapezoidProfile.Constraints constraints) {
    m_positionController = positionController;
    m_profileConstraints = constraints;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    SendableHelper.addChild(builder, this, m_positionController, "Inner Position Loop");
    builder.addDoubleProperty("Position State", () -> m_setpoint.position, null);
    builder.addDoubleProperty("Velocity State", () -> m_setpoint.velocity, null);
    builder.addDoubleProperty("Position Goal", () -> m_goal.position, null);
    builder.addDoubleProperty("Velocity Goal", () -> m_goal.velocity, null);
  }

  @Override
  public void setReference(double reference, double measurement, double feedforward) {
    if (reference != m_goal.position) {
      if (m_positionController.isContinuousInputEnabled()) {
        // Get error which is smallest distance between goal and measurement
        double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
        double goalMinDistance =
            MathUtil.inputModulus(m_goal.position - measurement, -errorBound, errorBound);
        double setpointMinDistance =
            MathUtil.inputModulus(m_setpoint.position - measurement, -errorBound, errorBound);

        // Recompute the profile goal with the smallest error, giving the shortest path. The goal
        // may be outside the input range after this operation, but that's OK because the controller
        // will still go there and report an error of zero. In other words, the setpoint only needs
        // to be offset from the measurement by the input range modulus; they don't need to be
        // equal.
        m_goal.position = goalMinDistance + measurement;
        m_setpoint.position = setpointMinDistance + measurement;
      }
      m_goal = new TrapezoidProfile.State(reference, 0);
    }

    var profile = new TrapezoidProfile(m_profileConstraints, m_goal, m_setpoint);
    m_setpoint = profile.calculate(m_dt);
    m_positionController.setReference(m_setpoint.position, measurement, feedforward);
  }

  @Override
  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_positionController.enableContinuousInput(minimumInput, maximumInput);
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
  }

  @Override
  public void disableContinuousInput() {
    m_positionController.disableContinuousInput();
  }

  @Override
  public boolean isContinuousInputEnabled() {
    return m_positionController.isContinuousInputEnabled();
  }
}
