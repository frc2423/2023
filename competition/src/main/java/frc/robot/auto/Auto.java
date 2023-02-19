package frc.robot.auto;

import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto extends StateMachine {

    StateMachine selectedAutonomous;
    StateMachine taxi;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public Auto() {
        super("run");
        taxi = new BasedAuto();

        m_chooser.setDefaultOption("taxi", "taxi");

        SmartDashboard.putData("Auto choices", m_chooser);
    }

    public void getAuto() {
        String name = m_chooser.getSelected();


        switch (name) {
            case "taxi":
                selectedAutonomous = taxi;
                break;
            default:
                selectedAutonomous = taxi;
                break;
        }
    }

    public void restart() {
        setState("run");
    }

    @State(name = "run")
    public void runState(StateContext ctx) {
        if (ctx.isInit()) {
            getAuto();
            selectedAutonomous.setState(selectedAutonomous.getDefaultState());
        }
        selectedAutonomous.run();
    }
}
