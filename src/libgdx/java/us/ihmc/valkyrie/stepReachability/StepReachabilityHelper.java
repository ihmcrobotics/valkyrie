package us.ihmc.valkyrie.stepReachability;

import java.util.function.Consumer;

import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;

public class StepReachabilityHelper
{
   private final Messager messager;

   public StepReachabilityHelper(Messager messager)
   {
      this.messager = messager;
   }

   public void subscribeToIKOutput(Consumer<Triple<Point3D, Quaternion, float[]>> consumer)
   {
      messager.addTopicListener(StepReachabilityMessagerAPI.IKOutput, ikOutput ->
      {
         IDLSequence.Float ikJointAngles = ikOutput.getDesiredJointAngles();
         if (ikJointAngles.isEmpty() || Double.isNaN(ikOutput.getSolutionQuality()))
         {
            LogTools.error("IK toolbox has no solution.");
            return;
         }

         float[] jointAnglesCopy = new float[ikJointAngles.size()];
         for (int i = 0; i < jointAnglesCopy.length; i++)
         {
            jointAnglesCopy[i] = ikJointAngles.get(i);
         }

         consumer.accept(Triple.of(ikOutput.getDesiredRootPosition(), ikOutput.getDesiredRootOrientation(), jointAnglesCopy));
      });
   }

   public <T> void publish(MessagerAPIFactory.Topic<T> topic, T message)
   {
      messager.submitMessage(topic, message);
   }
}
