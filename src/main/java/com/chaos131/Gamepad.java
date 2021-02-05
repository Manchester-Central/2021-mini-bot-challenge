package com.chaos131;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * Gamepad provides an interface to the controls expected on an XBox/PlayStation
 * style controller.  The layout of axes and buttons is controlled by file in
 * the deploy directory.  The constructor of Gamepad takes a parameter sepcifing
 * the file to check, and will look for a preset option to be specifed.
 * 
 * Currently, valid options are "type1" and "asdf"
 */
public class Gamepad extends GenericHID {

    public Button a;
    public Button b;
    public Button x;
    public Button y;

    public Button start;
    public Button select;

    public Button leftBumper;
    public Button rightBumper;
    public Button leftTrigger;
    public Button rightTrigger;
    public Button leftJoystick;
    public Button rightJoystick;

    public Button up;
    public Button right;
    public Button down;
    public Button left;

    private int leftXAxis;
    private int leftYAxis;
    private int rightXAxis;
    private int rightYAxis;

    private static final int DPAD_UP = 0;
    private static final int DPAD_RIGHT = 90;
    private static final int DPAD_DOWN = 180;
    private static final int DPAD_LEFT = 270;
    private static final double TRIGGER_THRESHOLD = -0.5;

    public Gamepad(int port, String LayoutPath) {
        super(port);

        String layout = "unknown";

        File layoutFile = new File(Filesystem.getDeployDirectory(), LayoutPath);
        try {
            Scanner scan = new Scanner(layoutFile);
            layout = scan.next().trim();
            scan.close();
            System.err.println("Found Layout " + layout);
        } catch (FileNotFoundException e) {
            System.err.println("Can not read from file " + layoutFile.getPath());
        }

        if (layout.compareTo("type1") == 0) {
            type1();
        } else if (layout.compareTo("type2") == 0) {
            type2();
        } else {
            System.err.println("Unrecongized layout " + layout);
            invalid();
        }

        up = new POVButton(this, DPAD_UP);
        right = new POVButton(this, DPAD_RIGHT);
        down = new POVButton(this, DPAD_DOWN);
        left = new POVButton(this, DPAD_LEFT);
    }

    private void type1() {
        a = new JoystickButton(this, 1);
        b = new JoystickButton(this, 2);
        x = new JoystickButton(this, 3);
        y = new JoystickButton(this, 4);
        start = new JoystickButton(this, 8);
        select = new JoystickButton(this, 7);
        leftBumper = new JoystickButton(this, 5);
        rightBumper = new JoystickButton(this, 6);
        leftTrigger = new Button(() -> getRawAxis(4) > TRIGGER_THRESHOLD);
        rightTrigger = new Button(() -> getRawAxis(5) > TRIGGER_THRESHOLD);
        leftJoystick = new JoystickButton(this, 9);
        rightJoystick = new JoystickButton(this, 10);
        leftXAxis = 0;
        leftYAxis = 1;
        rightXAxis = 2;
        rightYAxis = 3;
    }

    private void type2() {
        a = new JoystickButton(this, 2);
        b = new JoystickButton(this, 3);
        x = new JoystickButton(this, 1);
        y = new JoystickButton(this, 4);
        start = new JoystickButton(this, 10);
        select = new JoystickButton(this, 9);
        leftBumper = new JoystickButton(this, 5);
        rightBumper = new JoystickButton(this, 6);
        leftTrigger = new JoystickButton(this, 7);
        rightTrigger = new JoystickButton(this, 8);
        leftJoystick = new JoystickButton(this, 11);
        rightJoystick = new JoystickButton(this, 12);
        leftXAxis = 0;
        leftYAxis = 1;
        rightXAxis = 2;
        rightYAxis = 5;
    }

    private void invalid() {
        a = new Button(() -> false);
        b = new Button(() -> false);
        x = new Button(() -> false);
        y = new Button(() -> false);
        start = new Button(() -> false);
        select = new Button(() -> false);
        leftBumper = new Button(() -> false);
        rightBumper = new Button(() -> false);
        leftTrigger = new Button(() -> false);
        rightTrigger = new Button(() -> false);
        leftJoystick = new Button(() -> false);
        rightJoystick = new Button(() -> false);
        leftXAxis = 10;
        leftYAxis = 10;
        rightXAxis = 10;
        rightYAxis = 10;
    }

    public double getLeftX() {
        return getRawAxis(leftXAxis);
    }

    public double getLeftY() {
        return -getRawAxis(leftYAxis);
    }

    public double getRightX() {
        return getRawAxis(rightXAxis);
    }

    public double getRightY() {
        return -getRawAxis(rightYAxis);
    }

    private double calculateAngle(double x, double y) {
        return Math.toDegrees(Math.atan2(y, x));
    }

    public double getRightJoystickAngle() {
        return calculateAngle(getRightX(), getRightY());
    }

    public double getLeftJoystickAngle() {
        return calculateAngle(getLeftX(), getLeftY());
    }

    public double getX(GenericHID.Hand hand) {
        if (hand == Hand.kRight) {
            return getRightX();
        } else {
            return getLeftX();
        }
    }

    public double getY(GenericHID.Hand hand) {
        if (hand == Hand.kRight) {
            return getRightY();
        } else {
            return getLeftY();
        }
    }
}
