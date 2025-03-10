package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Wrapper for a controller that can have a set of bindings applied to it.
 */
public class Controller {
    private final int m_mainPort = 0;
    private final int m_assistPort = 1;
    CommandXboxController m_mainController = new CommandXboxController(m_mainPort);
    CommandXboxController m_assistController = new CommandXboxController(m_assistPort);
    ControllerBindings m_mainCurrentBindings;
    ControllerBindings m_assistCurrentBindings;

    public void applyBindings(ControllerBindings mainBindings) {
        if (m_mainCurrentBindings != null) {
            m_mainCurrentBindings.unbind(m_mainController);
        }

        m_mainController = new CommandXboxController(m_mainPort);

        if (mainBindings != null) {
            mainBindings.bind(m_mainController);
        }

        m_mainCurrentBindings = mainBindings;
    }

    public void applyBindings(ControllerBindings mainBindings, ControllerBindings assistBindings) {
        if (m_mainCurrentBindings != null) {
            m_mainCurrentBindings.unbind(m_mainController);
        }

        if (m_assistCurrentBindings != null) {
            m_assistCurrentBindings.unbind(m_assistController);
        }

        m_mainController = new CommandXboxController(m_mainPort);
        m_assistController = new CommandXboxController(m_assistPort);

        if (mainBindings != null) {
            mainBindings.bind(m_mainController);
        }

        if (assistBindings != null) {
            assistBindings.bind(m_assistController);
        }

        m_mainCurrentBindings = mainBindings;
        m_assistCurrentBindings = assistBindings;
    }

    public void clearBindings() {
        applyBindings(null, null);
    }
}
