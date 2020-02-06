/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorWheelSystem extends SubsystemBase {
  /**
   * Creates a new ColorWheelSystem.
   */
  I2C.Port i2cPort;
  ColorSensorV3 colorSensor;
  Color currentColor;

  Color8Bit YELLOW = new Color8Bit((int)(.32 *255), (int)(.13 *255), (int)(.56*255));
  Color8Bit RED = new Color8Bit((int)(.42*255), (int)(.4*255), (int)(.18*255));
  Color8Bit GREEN = new Color8Bit((int)(.2*255), (int)(.54*255), (int)(.26*255));
  Color8Bit BLUE = new Color8Bit((int)(.15*255), (int)(.44*255), (int)(.4*255));

  VictorSP colorWheel;
  double VARY = .1;


  public ColorWheelSystem(VictorSP cw) {
i2cPort = I2C.Port.kOnboard;
colorSensor = new ColorSensorV3(i2cPort);

currentColor = colorSensor.getColor();

colorWheel = cw;
 }

 boolean isColor(Color8Bit desiredColor) {
  double r = currentColor.red;
  double g = currentColor.green;
  double b = currentColor.blue;
  // zSystem.out.println(desiredColor.red);
  if (r > .4 && desiredColor == RED) {
      return true;
  } else if(b < .2 && desiredColor == YELLOW && g > .5){
      return true;
  }else {
      return Math.abs(r - desiredColor.red) < VARY && Math.abs(g - desiredColor.green) < VARY
              && Math.abs(b - desiredColor.blue) < VARY;
  }
}

public void showColor() {
  currentColor = colorSensor.getColor();
  // System.out.println(currentColor.red*255*2 + ", " + currentColor.green*255*2 +
  // ", " + currentColor.blue*2*255);
  if (isColor(BLUE))
      System.out.println("Blue");
  ;
  if (isColor(RED))
      System.out.println("Red");
  if (isColor(GREEN))
      System.out.println("Green");
  if (isColor(YELLOW))
      System.out.println("Yellow");
}

public void dispColor() {
  currentColor = colorSensor.getColor();
  SmartDashboard.putNumber("red", currentColor.red);
  SmartDashboard.putNumber("blue", currentColor.blue);
  SmartDashboard.putNumber("green", currentColor.green);
}

public void goToColor(Color8Bit desiredColor){

  if(!isColor(desiredColor)){
      colorWheel.set(.8);
  }else{
      colorWheel.set(0);
  }
  
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
