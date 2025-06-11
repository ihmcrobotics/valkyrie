package us.ihmc.valkyrie.perception;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeUpdateThread;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2TunedRigidBodyTransform;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.ImageSensorPublishThread;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionThread;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractionThread;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphUpdateThread;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

import javax.annotation.Nullable;
import java.util.Map;

public class ValkyrieAutonomyProcess
{
   private static final ValkyrieRobotModel ROBOT_MODEL = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);

   // ROS2
   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase() + "_node");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   // Robot
   private final ROS2SyncedRobotModel syncedRobot;
   private final RepeatingTaskThread robotUpdateThread;
   private final ROS2TunedRigidBodyTransform zedTunableTransform;
   private final ROS2TunedRigidBodyTransform realsenseTunableTransform;

   // ZED Stuff
   private final ROS2DemandGraphNode zedDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_ZED);
   private final ROS2DemandGraphNode zedPublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);

   private static final Map<Integer, ROS2Topic<? extends Packet<?>>> ZED_IMAGE_TOPIC_MAP
         = Map.of(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY, PerceptionAPI.SRT_ZED_LEFT_COLOR_STREAM_STATUS,
                  ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY, PerceptionAPI.SRT_ZED_RIGHT_COLOR_STREAM_STATUS,
                  ZEDImageSensor.DEPTH_IMAGE_KEY, PerceptionAPI.ZED_DEPTH);
   @Nullable
   private ImageSensorPublishThread zedPublishThread;
   @Nullable
   private ImageSensor zedSensor;

   // Detections
   private final DetectionManager detectionManager;

   // Scene Graph
   private ROS2SceneGraph sceneGraph;
   private ROS2SceneGraphUpdateThread sceneGraphUpdateThread;

   // Behaviors
   private final ROS2BehaviorTreeUpdateThread behaviorTreeUpdateThread;

   // YOLO
   private final ROS2DemandGraphNode yoloZEDDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_YOLO_ZED);
   private final ROS2DemandGraphNode yoloAnnotatedImageDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_YOLO_ANNOTATED_IMAGE);
   private YOLOv8DetectionThread yoloThread;

   // Planar Regions
   private final ROS2DemandGraphNode planarRegionsDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_PLANAR_REGIONS);
   private RapidPlanarRegionsExtractionThread planarRegionsThread;

   private ROS2PeerClockOffsetEstimator peerClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);

   public ValkyrieAutonomyProcess(@Nullable ImageSensor zedSensor)
   {
      // Robot
      syncedRobot = new ROS2SyncedRobotModel(ROBOT_MODEL, ros2Node);
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);
      zedTunableTransform = ROS2TunedRigidBodyTransform.toBeTuned(ros2Helper,
                                                                  PerceptionAPI.EXPERIMENTAL_CAMERA_TO_PARENT_TUNING,
                                                                  ROBOT_MODEL.getSensorInformation().getExperimentalCameraTransform());
      realsenseTunableTransform = ROS2TunedRigidBodyTransform.toBeTuned(ros2Helper,
                                                                        PerceptionAPI.STEPPING_CAMERA_TO_PARENT_TUNING,
                                                                        ROBOT_MODEL.getSensorInformation().getSteppingCameraTransform());
      robotUpdateThread = new RepeatingTaskThread("SyncedRobotUpdate", () ->
      {
         zedTunableTransform.update();
         realsenseTunableTransform.update();
         syncedRobot.update();
      }).setFrequencyLimit(30.0);
      robotUpdateThread.startRepeating();

      // Demand graph
      initializeDemandGraph();

      // Sensors
      if (zedSensor != null)
         initializeSensors(zedSensor);

      // Detections
      detectionManager = new DetectionManager(ros2Node);

      // Scene Graph
      initializeSceneGraph();

      // Behavior Tree
      behaviorTreeUpdateThread = new ROS2BehaviorTreeUpdateThread(ros2Node, peerClockOffsetEstimator, ROBOT_MODEL, sceneGraph, detectionManager);
      behaviorTreeUpdateThread.startRepeating();

      // YOLO
      if (zedSensor != null)
         initializeYOLO();

      // Planar Regions
      if (zedSensor != null)
         initializePlanarRegions();
   }

   private void initializeDemandGraph()
   {
      zedDemandNode.addDependents(zedPublishDemandNode, yoloZEDDemandNode);

      planarRegionsDemandNode.addDependents(yoloZEDDemandNode);
   }

   private void destroyDemandGraph()
   {
      zedDemandNode.destroy();
      zedPublishDemandNode.destroy();

      yoloZEDDemandNode.destroy();
      yoloAnnotatedImageDemandNode.destroy();

      planarRegionsDemandNode.destroy();
   }

   private void initializeSensors(ImageSensor zedSensor)
   {
      // ZED
      this.zedSensor = zedSensor;
      zedSensor.setSensorFrame(syncedRobot.getReferenceFrames().getExperimentalCameraFrame());
      loopOnDemand(zedSensor.getGrabThread(), zedDemandNode);
      zedPublishThread = new ImageSensorPublishThread(ros2Node, zedSensor);
      loopOnDemand(zedPublishThread, zedPublishDemandNode);
   }

   private void destroySensors()
   {
      if (zedPublishThread != null)
         zedPublishThread.blockingKill();

      if (zedSensor != null)
         zedSensor.close();
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

   private void initializeYOLO()
   {
      yoloThread = new YOLOv8DetectionThread(peerClockOffsetEstimator, yoloAnnotatedImageDemandNode::isDemanded);

      // Initialize demand node callbacks to work with ZED
      yoloZEDDemandNode.addDemandChangedCallback(isDemanded ->
                                                 {
                                                    if (isDemanded)
                                                    {  // YOLO demanded with ZED
                                                       yoloThread.setImageSensor(zedSensor, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY, ZEDImageSensor.DEPTH_IMAGE_KEY);
                                                       yoloThread.startRepeating();
                                                    }
                                                 });
      if (yoloZEDDemandNode.isDemanded())
      {
         yoloThread.setImageSensor(zedSensor, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY, ZEDImageSensor.DEPTH_IMAGE_KEY);
         yoloThread.startRepeating();
      }
   }

   private void initializePlanarRegions()
   {
      if (zedSensor != null)
      {
         planarRegionsThread = new RapidPlanarRegionsExtractionThread(ros2Node, new OpenCLManager(), zedSensor, ZEDImageSensor.DEPTH_IMAGE_KEY);
         loopOnDemand(planarRegionsThread, planarRegionsDemandNode);
      }
   }

   public void close()
   {
      System.out.println("Closing " + getClass().getSimpleName());
      try
      {
         destroyDemandGraph();

         destroySceneGraph();

         behaviorTreeUpdateThread.blockingKill();
         yoloThread.blockingKill();
         planarRegionsThread.blockingKill();

         destroySensors();

         robotUpdateThread.blockingKill();
         syncedRobot.destroy();

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