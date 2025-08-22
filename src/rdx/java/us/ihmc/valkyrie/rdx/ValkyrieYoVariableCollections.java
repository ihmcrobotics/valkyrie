package us.ihmc.valkyrie.rdx;

import us.ihmc.rdx.ui.yo.ImPlotYoGraphPanel;

public class ValkyrieYoVariableCollections
{
   public static void addYoVariablesToPanel(ImPlotYoGraphPanel yoGraphUI)
   {
      yoGraphUI.graphVariable("Dynamics simulation",
                              "AvatarSimulationFactoryContainer.main.DRCEstimatorThread.SensorReaderFactory.DRCPerfectSensorReader.SensorProcessing.monotonicTime");
      yoGraphUI.graphVariable("Kinematics simulation", "HumanoidKinematicsSimulationContainer.main.HumanoidKinematicsSimulation.time");
      yoGraphUI.graphVariable("Kinematics simulation", "HumanoidKinematicsSimulationContainer.main.HumanoidKinematicsSimulation.DRCControllerThread"
                                                       + ".DRCMomentumBasedController.HumanoidHighLevelControllerManager.WalkingControllerState.WholeBodyControllerCore"
                                                       + ".WholeBodyInverseDynamicsSolver.InverseDynamicsOptimizationControlModule.InverseDynamicsQPSolver.qpSolverTimerCount");
      yoGraphUI.graphVariable("Kinematics simulation", "HumanoidKinematicsSimulationContainer.main.HumanoidKinematicsSimulation.l_foot_SettableFootSwitch");
      yoGraphUI.graphVariable("Kinematics simulation", "HumanoidKinematicsSimulationContainer.main.HumanoidKinematicsSimulation.r_foot_SettableFootSwitch");
      yoGraphUI.graphVariable("Dynamics simulation",
                              "AvatarSimulationFactoryContainer.main.DRCEstimatorThread.SensorReaderFactory.DRCPerfectSensorReader.SensorProcessing.monotonicTime");
      yoGraphUI.graphVariable("Kinematics simulation", "HumanoidKinematicsSimulationContainer.main.HumanoidKinematicsSimulation.DRCControllerThread"
                                                       + ".DRCMomentumBasedController.HumanoidHighLevelControllerManager.WalkingControllerState.WholeBodyControllerCore"
                                                       + ".WholeBodyInverseDynamicsSolver.InverseDynamicsOptimizationControlModule.InverseDynamicsQPSolver.qpSolverTimerCount");
      yoGraphUI.graphVariable("Kinematics simulation", "HumanoidKinematicsSimulationContainer.main.HumanoidKinematicsSimulation.l_foot_SettableFootSwitch");
      yoGraphUI.graphVariable("Kinematics simulation", "HumanoidKinematicsSimulationContainer.main.HumanoidKinematicsSimulation.r_foot_SettableFootSwitch");
   }
}
