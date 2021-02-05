// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

enum ButtonType {
    A, B, X, Y
}

interface IControllerMapping {
    int getLeftXAxis();

    int getLeftYAxis();

    int getRightXAxis();

    int getRightYAxis();

    int getButtonIndex(ButtonType buttonType);
}

class XBoxControllerMapping implements IControllerMapping {

    @Override
    public int getLeftXAxis() {
        return 0;
    }

    @Override
    public int getLeftYAxis() {
        return 1;
    }

    @Override
    public int getRightXAxis() {
        return 4;
    }

    @Override
    public int getRightYAxis() {
        return 5;
    }

    @Override
    public int getButtonIndex(ButtonType buttonType) {
        switch (buttonType) {
            case A:
                return 1;
            case B:
                return 2;
            case X:
                return 3;
            case Y:
                return 4;
            // TODO Handle other cases
            default:
                return 0;
        }
    }

}

class LogitechControllerMapping implements IControllerMapping {

    @Override
    public int getLeftXAxis() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public int getLeftYAxis() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public int getRightXAxis() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public int getRightYAxis() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public int getButtonIndex(ButtonType buttonType) {
        // TODO Auto-generated method stub
        return 0;
    }

}

/** Add your docs here. */
public class Gamepad {
    private Joystick m_joystick;

    private Button m_aButton;
    private Button m_bButton;
    private Button m_xButton;
    private Button m_yButton;

    public Gamepad(int port) {
        m_joystick = new Joystick(port);
        m_aButton = createButton(ButtonType.A);
        m_bButton = createButton(ButtonType.B);
        m_xButton = createButton(ButtonType.X);
        m_yButton = createButton(ButtonType.Y);
    }

    private IControllerMapping getControllerMapping() {
        String joystickName = m_joystick.getName().toLowerCase();
        switch (joystickName) {
            case "xbox controller":
                return new XBoxControllerMapping();
            default:
                return new LogitechControllerMapping();
        }
    }

    public String getControllerName() {
        return m_joystick.getName();
    }

    public double getLeftX() {
        return m_joystick.getRawAxis(getControllerMapping().getLeftXAxis());
    }

    public double getLeftY() {
        return m_joystick.getRawAxis(getControllerMapping().getLeftYAxis());
    }

    public double getRightX() {
        return m_joystick.getRawAxis(getControllerMapping().getRightXAxis());
    }

    public double getRightY() {
        return m_joystick.getRawAxis(getControllerMapping().getRightYAxis());
    }

    public Button getAButton() {
        return m_aButton;
    }

    public Button getBButton() {
        return m_bButton;
    }

    public Button getXButton() {
        return m_xButton;
    }

    public Button getYButton() {
        return m_yButton;
    }

    private Button createButton(ButtonType buttonType) {
        return new Button() {
            @Override
            public boolean get() {
                return m_joystick.getRawButton(getControllerMapping().getButtonIndex(buttonType));
            }
        };
    }
}
