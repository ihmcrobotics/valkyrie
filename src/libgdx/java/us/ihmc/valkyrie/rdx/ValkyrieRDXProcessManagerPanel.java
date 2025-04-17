package us.ihmc.valkyrie.rdx;

import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.behaviors.simulation.EnvironmentInitialSetup;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.rdx.ui.RDXProcessManagerPanel;
import us.ihmc.rdx.ui.processes.RestartableProcess;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieRDXProcessManagerPanel extends RDXProcessManagerPanel
{
   private final ImInt robotVersion = new ImInt(2);
   private final ValkyrieRobotModel valkyrieRobotModel;
   private final String[] robotVersions = new String[ValkyrieRobotVersion.values().length];
   {
      ValkyrieRobotVersion[] values = ValkyrieRobotVersion.values();
      for (int i = 0; i < values.length; i++)
      {
         robotVersions[i] = values[i].name();
      }
   }

   private final RestartableProcess kinematicsSimulationProcess;

   public ValkyrieRDXProcessManagerPanel(ValkyrieRobotModel valkyrieRobotModel)
   {
      this(valkyrieRobotModel, new Pose3D());
   }

   public ValkyrieRDXProcessManagerPanel(ValkyrieRobotModel valkyrieRobotModel, Pose3DReadOnly startingPose)
   {
      super(startingPose);

      this.valkyrieRobotModel = valkyrieRobotModel;

      kinematicsSimulationProcess = new ValkyrieKinematicsSimulationProcess(logToFile::get, this::getRobotModel, environmentInitialSetup);

      processes.add(kinematicsSimulationProcess);
   }

   public ValkyrieRDXProcessManagerPanel(ValkyrieRobotModel valkyrieRobotModel, EnvironmentInitialSetup environmentInitialSetup)
   {
      super(environmentInitialSetup);

      this.valkyrieRobotModel = valkyrieRobotModel;

      kinematicsSimulationProcess = new ValkyrieKinematicsSimulationProcess(logToFile::get, this::getRobotModel, this.environmentInitialSetup);

      processes.add(kinematicsSimulationProcess);
   }

   @Override
   public ImInt getRobotVersion()
   {
      return robotVersion;
   }

   @Override
   public String[] getRobotVersions()
   {
      return robotVersions;
   }

   @Override
   protected ValkyrieRobotModel getRobotModel()
   {
      return valkyrieRobotModel;
   }

   public void dispose()
   {
      // Destroy in a reasonable order
      kinematicsSimulationProcess.destroy();

      super.dispose();
   }

   public void setRobotTarget(RobotTarget robotTarget)
   {
      this.robotTarget.set(robotTarget.ordinal());
   }

   public RestartableProcess getKinematicsSimulationProcess()
   {
      return kinematicsSimulationProcess;
   }
}
