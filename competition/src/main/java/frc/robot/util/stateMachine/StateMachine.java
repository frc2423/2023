package frc.robot.util.stateMachine;

import java.lang.reflect.Method;
import java.lang.annotation.Annotation;
import java.util.HashMap;

public class StateMachine {

  private HashMap<String, Method> states;
  private String state = "";
  private String nextState;
  private String defaultState;
  private StateContext ctx;

  public StateMachine(String defaultState) {
    this.defaultState = defaultState;
    states = new HashMap<String, Method>();
    Method[] methods = this.getClass().getDeclaredMethods(); // obtain all method objects
    for (Method method : methods) {
      Annotation[] annotations = method.getDeclaredAnnotations();
      for (Annotation annotation : annotations) {
        // Gathers each method that has State annotations
        if (annotation instanceof State) {
          State myAnnotation = (State) annotation;
          states.put(myAnnotation.name(), method);
        }
      }
    }
    setState(defaultState);
  }

  /**
   * Gets the current state of the state machine.
   */
  public String getState() {
    return this.state;
  }

  /**
   * Calls the function associated with the current state.
   * <p>
   * Run this method periodically.
   * </p>
   */
  public void run() {
    if (nextState != null) {
      this.ctx = new StateContext();
      state = nextState;
      nextState = null;
    }
    runState(this.state);
  }

  /**
   * Sets the current state.
   * 
   * @param name : String <i>name of the state</i>
   */
  public void setState(String name) {
    nextState = name;
  }

  public void runState(String name) {
    try {
      var stateMethod = states.get(name);
      if (stateMethod == null) {
        return;
      }
      if (stateMethod.getParameterCount() == 0) {
        stateMethod.setAccessible(true);
        stateMethod.invoke(this);
        
        if (ctx.isInit()) {
          ctx.initialize();
        }
      } else if (ctx != null) {
        stateMethod.setAccessible(true);
        stateMethod.invoke(this, ctx);
        if (ctx.isInit()) {
          ctx.initialize();
        }
      }
    } catch (Exception e) {
      e.printStackTrace();
      System.out.println(e + ": State for " + getClass().getName() + " -> " + name + " cause:" + e.getCause());
    }
  }

  public String getDefaultState() {
    return defaultState;
  }
}