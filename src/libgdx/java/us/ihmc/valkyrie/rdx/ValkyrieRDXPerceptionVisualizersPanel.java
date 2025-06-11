package us.ihmc.valkyrie.rdx;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.streaming.ROS2SRTVideoStreamImageMessageRelay;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXDetectionManagerSettings;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Visualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

/**
 * A common set of visualizers specific to sensors on Valkyrie
 */
public class ValkyrieRDXPerceptionVisualizersPanel extends RDXPerceptionVisualizersPanel
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2Helper ros2Helper;

   private final RDXROS2RobotVisualizer robotVisualizer;
   private final RDXROS2ColoredPointCloudVisualizer d455ColoredPointCloudVisualizer;
   private final RDXROS2ImageMessageVisualizer realsenseColorImageVisualizer;
   private final RDXROS2ImageMessageVisualizer realsenseDepthImageVisualizer;
   private final RDXROS2ColoredPointCloudVisualizer zed2ColoredPointCloudVisualizer;
   private final RDXROS2ImageMessageVisualizer zedLeftColorImageVisualizer;
   private final RDXROS2ImageMessageVisualizer zedRightColorImageVisualizer;
   private final RDXROS2ImageMessageVisualizer zed2DepthImageVisualizer;
   private final RDXROS2ImageMessageVisualizer yoloAnnotatedImageVisualizer;
   private final RDXROS2FramePlanarRegionsVisualizer planarRegionsVisualizer;
   private final RDXDetectionManagerSettings detectionManagerSettings;

   private final ROS2SRTVideoStreamImageMessageRelay videoStreamImageMessageRelay;

   public ValkyrieRDXPerceptionVisualizersPanel(RDXBaseUI baseUI,
                                                ROS2Node ros2Node,
                                                ROS2PeerClockOffsetEstimator ros2PeerClockOffsetEstimator,
                                                ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;

      ros2Helper = new ROS2Helper(ros2Node);

      videoStreamImageMessageRelay = new ROS2SRTVideoStreamImageMessageRelay(PerceptionAPI.SRT_STREAM_IMAGE_MESSAGE_TOPIC_PAIRS, ros2Node, CompressionType.UNCOMPRESSED);

      // Robot visualizer
      {
         robotVisualizer = new RDXROS2RobotVisualizer(ros2Helper, syncedRobot);
         robotVisualizer.setPinned(true);
         robotVisualizer.setActive(true);
         addVisualizer(robotVisualizer);
      }

      // Intel Realsense D455 colored point cloud visualizer
      {
         d455ColoredPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("D455 Colored Point Cloud",
                                                                                  ros2Node,
                                                                                  PerceptionAPI.D455_DEPTH_IMAGE,
                                                                                  PerceptionAPI.REALSENSE_COLOR_IMAGE_SRT);
         d455ColoredPointCloudVisualizer.setActive(true);
         d455ColoredPointCloudVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_REALSENSE_PUBLICATION);
         addVisualizer(d455ColoredPointCloudVisualizer);
      }

      // Intel Realsense D455 color image visualizer
      {
         realsenseColorImageVisualizer = new RDXROS2ImageMessageVisualizer("D455 Color Image", ros2Node, PerceptionAPI.REALSENSE_COLOR_IMAGE_SRT);
         realsenseColorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_REALSENSE_PUBLICATION);
         addVisualizer(realsenseColorImageVisualizer);
      }

      // Intel Realsense D455 depth image visualizer
      {
         realsenseDepthImageVisualizer = new RDXROS2ImageMessageVisualizer("D455 Depth Image", ros2Node, PerceptionAPI.D455_DEPTH_IMAGE);
         realsenseDepthImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_REALSENSE_PUBLICATION);
         addVisualizer(realsenseDepthImageVisualizer);
      }

      // ZED2 colored point cloud visualizer
      {
         zed2ColoredPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED 2 Colored Point Cloud",
                                                                                  ros2Node,
                                                                                  PerceptionAPI.ZED2_DEPTH,
                                                                                  PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
         zed2ColoredPointCloudVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
         zed2ColoredPointCloudVisualizer.setActive(true);
         addVisualizer(zed2ColoredPointCloudVisualizer);
      }

      // ZED left color visualizer
      {
         zedLeftColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Left",
                                                                         ros2Node,
                                                                         PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
         zedLeftColorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
         addVisualizer(zedLeftColorImageVisualizer);
      }

      // ZED 2 color right image visualizer
      {
         zedRightColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Right",
                                                                          ros2Node,
                                                                          PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.RIGHT));
         zedRightColorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
         addVisualizer(zedRightColorImageVisualizer);
      }

      // ZED 2 depth image visualizer
      {
         zed2DepthImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Depth Image", ros2Node, PerceptionAPI.ZED2_DEPTH);
         zed2DepthImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
         addVisualizer(zed2DepthImageVisualizer);
      }

      // YOLOv8 settings visualizer
      {
         RDXROS2YOLOv8Visualizer yoloVisualizer = new RDXROS2YOLOv8Visualizer("YOLOv8",
                                                                              ros2Node,
                                                                              ros2PeerClockOffsetEstimator,
                                                                              PerceptionAPI.YOLO_ANNOTATED_IMAGE);
         yoloVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_YOLO_ANNOTATED_IMAGE);
         addVisualizer(yoloVisualizer);
      }

      // YOLOv8 annotated image visualizer
      {
         yoloAnnotatedImageVisualizer = new RDXROS2ImageMessageVisualizer("YOLOv8 Annotated Image",
                                                                          ros2Node,
                                                                          PerceptionAPI.YOLO_ANNOTATED_IMAGE);
         yoloAnnotatedImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_YOLO_ANNOTATED_IMAGE);
         addVisualizer(yoloAnnotatedImageVisualizer);
      }

      // Planar regions visualizer
      {
         planarRegionsVisualizer = new RDXROS2FramePlanarRegionsVisualizer("Planar Regions", ros2Node, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS);
         planarRegionsVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_PLANAR_REGIONS);
         planarRegionsVisualizer.setActive(false);
         addVisualizer(planarRegionsVisualizer);
      }

      // Detection Manager Settings visualizer
      {
         detectionManagerSettings = new RDXDetectionManagerSettings("Detection Manager Settings", ros2Node);
         addVisualizer(detectionManagerSettings);
      }
   }

   @Override
   public void destroy()
   {
      super.destroy();
      videoStreamImageMessageRelay.destroy();
   }

   public RDXROS2RobotVisualizer getRobotVisualizer()
   {
      return robotVisualizer;
   }

   public RDXROS2ColoredPointCloudVisualizer getD455ColoredPointCloudVisualizer()
   {
      return d455ColoredPointCloudVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getRealsenseDepthImageVisualizer()
   {
      return realsenseDepthImageVisualizer;
   }

   public RDXROS2ColoredPointCloudVisualizer getZed2ColoredPointCloudVisualizer()
   {
      return zed2ColoredPointCloudVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getZedLeftColorImageVisualizer()
   {
      return zedLeftColorImageVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getZedRightColorImageVisualizer()
   {
      return zedRightColorImageVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getZed2DepthImageVisualizer()
   {
      return zed2DepthImageVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getYoloAnnotatedImageVisualizer()
   {
      return yoloAnnotatedImageVisualizer;
   }

   public RDXROS2FramePlanarRegionsVisualizer getPlanarRegionsVisualizer()
   {
      return planarRegionsVisualizer;
   }
}

