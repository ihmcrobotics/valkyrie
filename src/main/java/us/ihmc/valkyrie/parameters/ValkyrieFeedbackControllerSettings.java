package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerFilterFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashSet;
import java.util.Set;

public class ValkyrieFeedbackControllerSettings implements FeedbackControllerSettings
{
   private YoDouble pelvisVelocityErrorBreakFrequency;
   private YoDouble chestVelocityErrorBreakFrequency;
   private YoDouble footVelocityErrorBreakFrequency;
   private YoDouble armJointsVelocityErrorBreakFrequency;

   private final String pelvisName;
   private final String chestName;
   private final Set<String> footNames;
   private final Set<String> armJointNames;

   public ValkyrieFeedbackControllerSettings(ValkyrieJointMap jointMap, RobotTarget target)
   {
      pelvisName = jointMap.getPelvisName();
      chestName = jointMap.getChestName();
      footNames = new HashSet<>(jointMap.getFootNames());
      armJointNames = new HashSet<>(jointMap.getArmJointNamesAsStrings());
   }

   @Override
   public boolean enableIntegralTerm()
   {
      return false; // Saves about 130 YoVariables.
   }

   @Override
   public FilterDouble1D getVelocity1DErrorFilter(String jointName, double dt, YoRegistry registry)
   {
      if (armJointNames.contains(jointName))
      {
         if (armJointsVelocityErrorBreakFrequency == null)
         {
            armJointsVelocityErrorBreakFrequency = new YoDouble(jointName + "VelocityErrorBreakFrequency", registry);
            armJointsVelocityErrorBreakFrequency.set(25.0);
         }

         return FeedbackControllerFilterFactory.createVelocity1DErrorLPFFilter(jointName, armJointsVelocityErrorBreakFrequency, dt, registry);
      }
      return null;
   }

   @Override
   public FilterVector3D getAngularVelocity3DErrorFilter(String endEffectorName, double dt, YoRegistry registry)
   {
      if (endEffectorName.equals(pelvisName))
      {
         if (pelvisVelocityErrorBreakFrequency == null)
         {
            pelvisVelocityErrorBreakFrequency = new YoDouble(pelvisName + "VelocityErrorBreakFrequency", registry);
            pelvisVelocityErrorBreakFrequency.set(25.0);
         }

         return FeedbackControllerFilterFactory.createAngularVelocityErrorLPFFilter(endEffectorName, pelvisVelocityErrorBreakFrequency, dt, registry);
      }

      if (endEffectorName.equals(chestName))
      {
         if (chestVelocityErrorBreakFrequency == null)
         {
            chestVelocityErrorBreakFrequency = new YoDouble(chestName + "VelocityErrorBreakFrequency", registry);
            chestVelocityErrorBreakFrequency.set(16.0);
         }

         return FeedbackControllerFilterFactory.createAngularVelocityErrorLPFFilter(endEffectorName, chestVelocityErrorBreakFrequency, dt, registry);
      }

      if (footNames.contains(endEffectorName))
      {
         if (footVelocityErrorBreakFrequency == null)
         {
            footVelocityErrorBreakFrequency = new YoDouble(endEffectorName + "VelocityErrorBreakFrequency", registry);
            footVelocityErrorBreakFrequency.set(16.0);
         }

         return FeedbackControllerFilterFactory.createAngularVelocityErrorLPFFilter(endEffectorName, footVelocityErrorBreakFrequency, dt, registry);
      }

      return null;
   }

   @Override
   public FilterVector3D getLinearVelocity3DErrorFilter(String endEffectorName, double dt, YoRegistry registry)
   {
      if (footNames.contains(endEffectorName))
      {
         if (footVelocityErrorBreakFrequency == null)
         {
            footVelocityErrorBreakFrequency = new YoDouble(endEffectorName + "VelocityErrorBreakFrequency", registry);
            footVelocityErrorBreakFrequency.set(16.0);
         }

         return FeedbackControllerFilterFactory.createLinearVelocityErrorLPFFilter(endEffectorName, footVelocityErrorBreakFrequency, dt, registry);
      }

      return null;
   }
}
