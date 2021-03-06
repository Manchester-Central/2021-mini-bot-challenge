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

    public static final double kRamseteB = 78.74;
    public static final double kRamseteZeta = 27.6;
    
    //TODO Figure out voltage
    public static final double ksVolts = 0.929; 
    public static final double kvVoltSecondsPerInch = 0.1608;
    public static final double kaVoltSecondsSquaredPerInch = 0.0009881;

    public static final double kRamseteP = 0.085;
    public static final double kRamseteI = 0;
    public static final double kRamseteD = 0;

}
