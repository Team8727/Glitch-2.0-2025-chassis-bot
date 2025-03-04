package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Wrapper for a controller that can have a set of bindings applied to it.
 */
public class Controller {
    CommandXboxController m_controller = new CommandXboxController(0);
    ControllerBindings m_currentBindings;

    public void applyBindings(ControllerBindings bindings) {
        if (m_currentBindings != null) {
            m_currentBindings.unbind(m_controller);
        }

        if (bindings != null) {
            bindings.bind(m_controller);
        }

        m_currentBindings = bindings;
    }

    public void clearBindings() {
        applyBindings(null);
    }
}
