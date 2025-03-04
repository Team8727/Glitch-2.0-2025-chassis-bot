package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Wrapper for a controller that can have a set of bindings applied to it.
 */
public class Controller {
    private final int m_port = 0;
    CommandXboxController m_controller = new CommandXboxController(m_port);
    ControllerBindings m_currentBindings;

    public void applyBindings(ControllerBindings bindings) {
        if (m_currentBindings != null) {
            m_currentBindings.unbind(m_controller);
        }

        m_controller = new CommandXboxController(m_port);

        if (bindings != null) {
            bindings.bind(m_controller);
        }

        m_currentBindings = bindings;
    }

    public void clearBindings() {
        applyBindings(null);
    }
}
