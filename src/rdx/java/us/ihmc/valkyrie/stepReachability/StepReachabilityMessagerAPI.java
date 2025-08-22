package us.ihmc.valkyrie.stepReachability;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.multiContact.KinematicsToolboxSnapshotDescription;
import us.ihmc.messager.MessagerAPIFactory;

public class StepReachabilityMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category Root = apiFactory.createRootCategory("StepReachabilityRoot");
   private static final MessagerAPIFactory.CategoryTheme Theme = apiFactory.createCategoryTheme("StepReachability");

   public static final MessagerAPIFactory.Topic<Boolean> SendTrajectory = topic("SendTrajectory");
   public static final MessagerAPIFactory.Topic<KinematicsToolboxOutputStatus> SendFrame = topic("SendFrame");
   public static final MessagerAPIFactory.Topic<KinematicsToolboxSnapshotDescription> StartIK = topic("StartIK");
   public static final MessagerAPIFactory.Topic<Boolean> StopIK = topic("StopIK");
   public static final MessagerAPIFactory.Topic<KinematicsToolboxOutputStatus> IKOutput = topic("KinematicsToolboxOutput");
   public static final MessagerAPIFactory.Topic<KinematicsToolboxSnapshotDescription> CurrentSnapshotDescription = topic("CurrentSnapshotDescription");

   public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return Root.child(Theme).topic(apiFactory.createTypedTopicTheme(name));
   }
}
