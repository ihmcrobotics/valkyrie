package us.ihmc.valkyrie.rdx;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.*;

public class ValkyrieKinematicSimulation
{
   public static HumanoidKinematicsSimulation create(ValkyrieRobotModel robotModel, HumanoidKinematicsSimulationParameters kinematicsSimulationParameters)
   {
      return create(robotModel, kinematicsSimulationParameters, false);
   }

   public static HumanoidKinematicsSimulation createForPreviews(ValkyrieRobotModel robotModel, HumanoidKinematicsSimulationParameters kinematicsSimulationParameters)
   {
      return create(robotModel, kinematicsSimulationParameters, true);
   }

   private static HumanoidKinematicsSimulation create(ValkyrieRobotModel robotModel,
                                                      HumanoidKinematicsSimulationParameters kinematicsSimulationParameters,
                                                      boolean createForPreviews)
   {
      ValkyrieWalkingControllerParameters walkingControllerParameters = (ValkyrieWalkingControllerParameters) robotModel.getWalkingControllerParameters();
      walkingControllerParameters.setDoPrepareManipulationForLocomotion(false);
      walkingControllerParameters.setSteppingParameters(new ValkyrieKinematicSteppingParameters(robotModel.getJointMap(),
                                                                                                robotModel.getRobotPhysicalProperties(),
                                                                                                robotModel.getTarget()));
      walkingControllerParameters.setSwingTrajectoryParameters(new ValkyrieKinematicSwingTrajectoryParameters(robotModel.getRobotPhysicalProperties(),
                                                                                                              robotModel.getTarget()));
      return createForPreviews ?
            HumanoidKinematicsSimulation.createForPreviews(robotModel, kinematicsSimulationParameters) :
            HumanoidKinematicsSimulation.create(robotModel, kinematicsSimulationParameters);
   }

   static class ValkyrieKinematicSteppingParameters extends ValkyrieSteppingParameters
   {
      private final ValkyrieJointMap jointMap;

      public ValkyrieKinematicSteppingParameters(ValkyrieJointMap jointMap, ValkyriePhysicalProperties physicalProperties, RobotTarget robotTarget)
      {
         super(physicalProperties, robotTarget);
         this.jointMap = jointMap;
      }
   }

   static class ValkyrieKinematicSwingTrajectoryParameters extends ValkyrieSwingTrajectoryParameters
   {
      public ValkyrieKinematicSwingTrajectoryParameters(ValkyriePhysicalProperties physicalProperties, RobotTarget target)
      {
         super(physicalProperties, target);
      }

      @Override
      public boolean addOrientationMidpointForObstacleClearance()
      {
         return true;
      }
   }

   public static void main(String[] args)
   {
      HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
      ValkyrieKinematicSimulation.create(new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.ARM_MASS_SIM), kinematicsSimulationParameters);
   }
}
