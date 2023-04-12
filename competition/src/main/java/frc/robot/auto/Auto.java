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
    StateMachine testauto;
    // StateMachine balanceauto;
    StateMachine gyroauto;
    StateMachine yoyoyo;
    StateMachine scoreonly; 
    StateMachine twopeice;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public Auto() {
        super("run");
        taxi = new BasedAuto();
        yoyo = new YoYoAuto();
        testauto = new TestAuto();
        // balanceauto = new GyroAuto();
        gyroauto = new GyroAuto();
        yoyoyo = new YoYoYoAuto();
        scoreonly = new ScoreOnly();
        twopeice = new TwoPeiceAuto();

        m_chooser.setDefaultOption("taxi", "taxi");
        m_chooser.addOption("yoyo", "yoyo");
        m_chooser.addOption("testauto", "testauto");
        // m_chooser.addOption("balanceauto", "balanceauto");
        m_chooser.addOption("gyroauto", "gyroauto");
        m_chooser.addOption("YoYoYoAuto" , "YoYoYoAuto");
        m_chooser.addOption("scoreonly", "scoreonly");
        m_chooser.addOption("twopeiceTEST", "twopeice");

        SmartDashboard.putData("Auto choices", m_chooser);
    }

    public void getAuto() {
        String name = m_chooser.getSelected();


        switch (name) {
            case "YoYoYoAuto":
                selectedAutonomous = yoyoyo;
                break;
            case "taxi":
                selectedAutonomous = taxi;
                break;
            case "yoyo":
                selectedAutonomous = yoyo;
                break;
            case "testauto":
                selectedAutonomous = testauto;
                break;
            // case "balanceauto":
            //     selectedAutonomous = balanceauto;
            //     break;
            case "gyroauto":
                selectedAutonomous = gyroauto;
                break;
            case "scoreonly":
                selectedAutonomous = scoreonly;
                break;
            case "twopeice":
                selectedAutonomous = twopeice;
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
