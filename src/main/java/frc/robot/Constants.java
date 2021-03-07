// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;
    
    // TODO: Figure out values.
    // Values currently shameless stolen from Team #195: https://github.com/frcteam195/MiniBot2021/blob/af77ab203140e34cfb80d830527c5bf72cb8ecf0/src/main/java/frc/robot/Constants.java#L20
    public static final double ksVolts = 0.929; 
    public static final double kvVoltSecondsPerMeter = 6.33;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0389;

    public static final double kRamseteP = 0.085;
    public static final double kRamseteI = 0;
    public static final double kRamseteD = 0;

}
