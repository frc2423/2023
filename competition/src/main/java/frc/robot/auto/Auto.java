package frc.robot.auto;

import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto extends StateMachine {

    StateMachine selectedAutonomous;
    StateMachine taxi;
    StateMachine yoyo;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public Auto() {
        super("run");
        taxi = new BasedAuto();
        yoyo = new YoYoAuto();

        m_chooser.setDefaultOption("taxi", "taxi");
        m_chooser.addOption("yoyo", "yoyo");

        SmartDashboard.putData("Auto choices", m_chooser);
    }

    public void getAuto() {
        String name = m_chooser.getSelected();


        switch (name) {
            case "taxi":
                selectedAutonomous = taxi;
                break;
            case "yoyo":
                selectedAutonomous = yoyo;
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
