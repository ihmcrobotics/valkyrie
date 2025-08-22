package us.ihmc.valkyrie.rdx;

import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.behaviors.simulation.EnvironmentInitialSetup;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.ui.processes.RestartableProcess;
import us.ihmc.valkyrie.ValkyrieRobotModel;

import java.util.function.Supplier;

public class ValkyrieKinematicsSimulationProcess extends RestartableProcess
{
   private final Supplier<Boolean> logToFile;
   private final Supplier<ValkyrieRobotModel> robotModelSupplier;
   private final EnvironmentInitialSetup environmentInitialSetup;
   private HumanoidKinematicsSimulation kinematicsSimulation;
   private final HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();

   public ValkyrieKinematicsSimulationProcess(Supplier<Boolean> logToFile,
                                              Supplier<ValkyrieRobotModel> robotModelSupplier,
                                              EnvironmentInitialSetup environmentInitialSetup)
   {
      this.logToFile = logToFile;
      this.robotModelSupplier = robotModelSupplier;
      this.environmentInitialSetup = environmentInitialSetup;
   }

   @Override
   protected void startInternal()
   {
      LogTools.info("Creating kinematics simulation");
      HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
      kinematicsSimulationParameters.setLogToFile(logToFile.get());
      kinematicsSimulationParameters.setCreateYoVariableServer(true);
      kinematicsSimulationParameters.setInitialGroundHeight(environmentInitialSetup.getGroundZ());
      kinematicsSimulationParameters.setInitialRobotYaw(environmentInitialSetup.getInitialYaw());
      kinematicsSimulationParameters.setInitialRobotX(environmentInitialSetup.getInitialX());
      kinematicsSimulationParameters.setInitialRobotY(environmentInitialSetup.getInitialY());
      kinematicsSimulation = ValkyrieKinematicSimulation.create(robotModelSupplier.get(), kinematicsSimulationParameters);
   }

   @Override
   protected void stopInternal()
   {
      kinematicsSimulation.destroy();
      kinematicsSimulation = null;
   }

   @Override
   public String getName()
   {
      return "Kinematics simulation";
   }

   public HumanoidKinematicsSimulationParameters getKinematicsSimulationParameters()
   {
      return kinematicsSimulationParameters;
   }
}
