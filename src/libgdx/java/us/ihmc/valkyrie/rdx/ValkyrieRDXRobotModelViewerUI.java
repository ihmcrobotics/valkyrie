package us.ihmc.valkyrie.rdx;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.rdx.ui.modelViewer.RDXRobotModelViewer;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSimulationCollisionModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieRDXRobotModelViewerUI
{
   public ValkyrieRDXRobotModelViewerUI()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.ARM_MASS_SIM);
      ValkyrieSimulationCollisionModel simulationCollisionModel = new ValkyrieSimulationCollisionModel(robotModel.getJointMap(), true);
      new RDXRobotModelViewer(robotModel, simulationCollisionModel);
   }

   public static void main(String[] args)
   {
      new ValkyrieRDXRobotModelViewerUI();
   }
}
