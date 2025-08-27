package us.ihmc.valkyrie.perception;

import us.ihmc.avatar.drcRobot.ROS2HumanoidFrames;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2NodeBuilder.SpecialTransportMode;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieAutonomyProcess
{
   // ROS2
   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase() + "_node");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
   private final ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);

   private final ROS2Node ros2LoopbackNode = new ROS2NodeBuilder().specialTransportMode(SpecialTransportMode.UDPV4_LOOPBACK_ADDRESS_ONLY)
                                                                  .build(getClass().getSimpleName().toLowerCase() + "_loopback_node");

   // Robot
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2HumanoidFrames ros2HumanoidFrames;
   private final RepeatingTaskThread robotUpdateThread;
   private final ROS2TunedRigidBodyTransform zedTunableTransform;

   // ZED Stuff
   private final ROS2DemandGraphNode zedPublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
   private ImageSensorPublishThread zedPublishThread;
   private ImageSensorPublishThread zedLoopbackPublishThread;
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

   public ValkyrieAutonomyProcess(ValkyrieRobotModel robotModel, ImageSensor zedSensor)
   {
      // Robot
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);

      ros2HumanoidFrames = new ROS2HumanoidFrames(ros2Node, syncedRobot);

      zedTunableTransform = ROS2TunedRigidBodyTransform.toBeTuned(ros2Helper,
                                                                  PerceptionAPI.EXPERIMENTAL_CAMERA_TO_PARENT_TUNING,
                                                                  robotModel.getSensorInformation().getExperimentalCameraTransform());

      robotUpdateThread = new RepeatingTaskThread("SyncedRobotUpdate", () ->
      {
         zedTunableTransform.update();
         syncedRobot.update();
         ros2HumanoidFrames.update();
      }).setFrequencyLimit(60.0);
      robotUpdateThread.startRepeating();

      // Sensors
      initializeSensors(zedSensor);

      // Detections
      detectionManager = new DetectionManager(ros2Node);

      // Scene Graph
      initializeSceneGraph();

      // Behavior Tree
      behaviorTreeUpdateThread = new ROS2BehaviorTreeUpdateThread(ros2Node, ros2ClockOffsetEstimator, robotModel, sceneGraph, detectionManager);
      behaviorTreeUpdateThread.startRepeating();

      // YOLO
      initializeYOLO();

      // Planar Regions
      initializePlanarRegions();
   }

   private void destroyDemandGraph()
   {
      zedPublishDemandNode.destroy();

      yoloZEDDemandNode.destroy();
      yoloAnnotatedImageDemandNode.destroy();

      planarRegionsDemandNode.destroy();
   }

   private void initializeSensors(ImageSensor zedSensor)
   {
      // ZED
      this.zedSensor = zedSensor;
      zedSensor.setSensorFrame(ros2HumanoidFrames.getROS2FrameCopy(syncedRobot.getReferenceFrames().getExperimentalCameraFrame()));
      zedSensor.run(true); // Always start ZED, do not wait for any demand node

      zedPublishThread = new ImageSensorPublishThread(ros2Node, zedSensor);
      zedPublishThread.addTopic(PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.LEFT), ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      zedPublishThread.addTopic(PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.RIGHT), ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);
      zedPublishThread.addTopic(PerceptionAPI.ZED_DEPTH, ZEDImageSensor.DEPTH_IMAGE_KEY);
      loopOnDemand(zedPublishThread, zedPublishDemandNode);

      zedLoopbackPublishThread = new ImageSensorPublishThread(ros2LoopbackNode, zedSensor);
      zedLoopbackPublishThread.enableROS2Frames(true);
      zedLoopbackPublishThread.setCameraInfoPublishGrabSkipCount(5); // publish camera info once every 5 images
      zedLoopbackPublishThread.addTopic(PerceptionAPI.ROS2_ZED_COLOR_IMAGES.get(RobotSide.LEFT), ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      zedLoopbackPublishThread.addTopic(PerceptionAPI.ROS2_ZED_COLOR_IMAGES.get(RobotSide.RIGHT), ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);
      zedLoopbackPublishThread.addTopic(PerceptionAPI.ROS2_ZED_COLOR_CAMERA_INFOS.get(RobotSide.LEFT), ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      zedLoopbackPublishThread.addTopic(PerceptionAPI.ROS2_ZED_COLOR_CAMERA_INFOS.get(RobotSide.RIGHT), ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);
      zedLoopbackPublishThread.addTopic(PerceptionAPI.ROS2_ZED_DEPTH_IMAGE, ZEDImageSensor.DEPTH_IMAGE_KEY);
      zedLoopbackPublishThread.addTopic(PerceptionAPI.ROS2_ZED_DEPTH_CAMERA_INFO, ZEDImageSensor.DEPTH_IMAGE_KEY);
      loopOnDemand(zedLoopbackPublishThread, zedPublishDemandNode);
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
      yoloThread = new YOLOv8DetectionThread(ros2ClockOffsetEstimator, yoloAnnotatedImageDemandNode::isDemanded);
      yoloThread.addDetectionConsumerCallback(detectionManager::addDetections);

      // Initialize demand node callbacks to work with both ZED
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
         ros2HumanoidFrames.remove();
         syncedRobot.destroy();

         ros2ClockOffsetEstimator.destroy();
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