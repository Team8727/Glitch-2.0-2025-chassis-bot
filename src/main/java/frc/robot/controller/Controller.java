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
    ControllerBindings m_currentBindings;

    public void applyBindings(ControllerBindings bindings) {
        if (m_currentBindings != null) {
            m_currentBindings.unbind(m_mainController);
            m_currentBindings.unbind(m_assistController);
        }

        m_mainController = new CommandXboxController(m_mainPort);

        if (bindings != null) {
            bindings.bind(m_mainController);
            bindings.bind(m_assistController);
        }

        m_currentBindings = bindings;
    }

    public void clearBindings() {
        applyBindings(null);
    }
}
