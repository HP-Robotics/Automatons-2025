// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  public LEDPattern m_sidePattern = LEDConstants.defaultSidePattern;
  public LEDPattern m_middlePattern = LEDConstants.defaultMiddlePattern;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private final AddressableLEDBufferView m_leftView;
  private final AddressableLEDBufferView m_rightView;
  private final AddressableLEDBufferView m_middleView;

  public LEDSubsystem() {
    m_led = new AddressableLED(LEDConstants.port);
    m_buffer = new AddressableLEDBuffer(LEDConstants.length);
    m_leftView = m_buffer.createView(0, LEDConstants.middleFirstIndex - 1);
    m_rightView = m_buffer.createView(LEDConstants.middleLastIndex + 1, LEDConstants.length - 1).reversed();
    m_middleView = m_buffer.createView(LEDConstants.middleFirstIndex, LEDConstants.middleLastIndex);

    m_led.setLength(m_buffer.getLength());
    m_led.setData(m_buffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    m_sidePattern.applyTo(m_leftView);
    m_sidePattern.applyTo(m_rightView);
    m_middlePattern.applyTo(m_middleView);

    m_led.setData(m_buffer);
  }

  public Command SetSidePattern(LEDPattern pattern) {
    return (new InstantCommand(() -> m_sidePattern = pattern));
  }

  public Command SetMiddlePattern(LEDPattern pattern) {
    return (new InstantCommand(() -> m_middlePattern = pattern));
  }
}
