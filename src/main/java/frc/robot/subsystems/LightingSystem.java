/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotMap;

public class LightingSystem extends SubsystemBase {
  /**
   * Creates a new IntakeSystem.
   */
  public AddressableLED LEDs;
  public AddressableLEDBuffer buffer;
  private int length;

  public LightingSystem(int length) {
    LEDs = new AddressableLED(RobotMap.LED_LIGHTS.getPin());
    buffer = new AddressableLEDBuffer(length);
    this.length = length;
  }

  @Override
  public void periodic() {}

  public AddressableLED getLED() {
    return LEDs;
  }

  public void setColor(int d) {
    //gotta do presets and stuff manually. Gonna be a little time
    for (int i = 0; i < length; i++) {
      buffer.setHSV(i, d, 100, 100);
    }
    LEDs.setData(buffer);
  }
  public void setRandom(int min, int max) {
    //gotta do presets and stuff manually. Gonna be a little time
    for (int i = 0; i < length; i++) {
      buffer.setHSV(i, (int)(Math.random() * (max + min)) + min, 100, 100);
    }
    LEDs.setData(buffer);
  }
}