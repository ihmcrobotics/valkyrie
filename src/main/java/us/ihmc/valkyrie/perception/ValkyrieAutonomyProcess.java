package us.ihmc.valkyrie.perception;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeUpdateThread;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.ImageSensorPublishThread;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphUpdateThread;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieAutonomyProcess
{
   private static final ValkyrieRobotModel ROBOT_MODEL = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.ARM_MASS_SIM);

   // ROS2
   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase() + "_node");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
   private final ROS2PeerClockOffsetEstimator ros2PeerClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);

   // Robot
   private final ROS2SyncedRobotModel syncedRobot;
   private final RepeatingTaskThread robotUpdateThread;

   // Detections
   private final DetectionManager detectionManager;

   // Scene Graph
   private ROS2SceneGraph sceneGraph;
   private ROS2SceneGraphUpdateThread sceneGraphUpdateThread;

   // Behaviors
   private final ROS2BehaviorTreeUpdateThread behaviorTreeUpdateThread;

   // ZED
   private final ZEDImageSensor zedImageSensor;
   private final ImageSensorPublishThread zedPublishThread;

   public ValkyrieAutonomyProcess(ZEDImageSensor zedImageSensor)
   {
      // Robot
      syncedRobot = new ROS2SyncedRobotModel(ROBOT_MODEL, ros2Node);
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);
      robotUpdateThread = new RepeatingTaskThread("SyncedRobotUpdate", () ->
      {
         syncedRobot.update();
      }).setFrequencyLimit(30.0);
      robotUpdateThread.startRepeating();

      // Detections
      detectionManager = new DetectionManager(ros2Node);

      // Scene Graph
      initializeSceneGraph();

      // Behavior Tree
      behaviorTreeUpdateThread = new ROS2BehaviorTreeUpdateThread(ros2Node, ros2PeerClockOffsetEstimator, ROBOT_MODEL, sceneGraph, detectionManager);
      behaviorTreeUpdateThread.startRepeating();

      // ZED
      this.zedImageSensor = zedImageSensor;
      zedImageSensor.run(true);
      zedPublishThread = new ImageSensorPublishThread(ros2Node, zedImageSensor);
      zedPublishThread.addTopic(PerceptionAPI.SRT_ZED_LEFT_COLOR_STREAM_STATUS, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      zedPublishThread.addTopic(PerceptionAPI.SRT_ZED_RIGHT_COLOR_STREAM_STATUS, ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);
      zedPublishThread.addTopic(PerceptionAPI.ZED2_DEPTH, ZEDImageSensor.DEPTH_IMAGE_KEY);
      zedPublishThread.startRepeating();
   }

   private void initializeSceneGraph()
   {
      sceneGraph = new ROS2SceneGraph(ros2Helper);
      sceneGraphUpdateThread = new ROS2SceneGraphUpdateThread(sceneGraph, detectionManager, syncedRobot.getReferenceFrames()::getPelvisZUpFrame);
      sceneGraphUpdateThread.startRepeating();
   }

   private void destroySceneGraph()
   {
      sceneGraphUpdateThread.blockingKill();
      sceneGraph.destroy();
   }

   public void close()
   {
      System.out.println("Closing " + getClass().getSimpleName());
      try
      {
         destroySceneGraph();

         behaviorTreeUpdateThread.blockingKill();

         robotUpdateThread.blockingKill();
         syncedRobot.destroy();

         ros2PeerClockOffsetEstimator.destroy();
         ros2Node.destroy();
      }
      catch (Exception exception)
      {
         System.out.println("Exception thrown while closing:\n" + exception.getMessage());
      }
      System.out.println("Closed " + getClass().getSimpleName());
   }

   private static void loopOnDemand(RepeatingTaskThread loopThread, ROS2DemandGraphNode demandNode)
   {
      if (!loopThread.isAlive())
         loopThread.start();

      if (demandNode.isDemanded())
         loopThread.startRepeating();

      demandNode.addDemandChangedCallback(loopThread::setRepeating);
   }
}
