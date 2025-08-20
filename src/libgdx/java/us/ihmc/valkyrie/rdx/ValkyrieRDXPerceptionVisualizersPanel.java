package us.ihmc.valkyrie.rdx;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.rdx.ui.graphics.RDXRobotPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXDetectionManagerSettings;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Visualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

/**
 * A common set of visualizers specific to sensors on Valkyrie
 */
public class ValkyrieRDXPerceptionVisualizersPanel extends RDXRobotPerceptionVisualizersPanel
{
   public ValkyrieRDXPerceptionVisualizersPanel(ROS2Node ros2Node,
                                                ROS2PeerClockOffsetEstimator ros2PeerClockOffsetEstimator,
                                                ROS2SyncedRobotModel syncedRobot)
   {
      super(ros2Node, syncedRobot, ros2PeerClockOffsetEstimator);

      // Intel Realsense D455 colored point cloud visualizer
      {
         realsenseColoredPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("D455 Colored Point Cloud",
                                                                                  ros2Node,
                                                                                  PerceptionAPI.D455_DEPTH_IMAGE,
                                                                                  PerceptionAPI.REALSENSE_COLOR_IMAGE_SRT);
         realsenseColoredPointCloudVisualizer.setActive(true);
         realsenseColoredPointCloudVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_REALSENSE_PUBLICATION);
         addVisualizer(realsenseColoredPointCloudVisualizer);
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
         zedColoredPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED 2 Colored Point Cloud",
                                                                                 ros2Node,
                                                                                 PerceptionAPI.ZED_DEPTH,
                                                                                 PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.LEFT));
         zedColoredPointCloudVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
         zedColoredPointCloudVisualizer.setActive(true);
         addVisualizer(zedColoredPointCloudVisualizer);
      }

      // ZED left color visualizer
      {
         zedLeftColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Left",
                                                                         ros2Node,
                                                                         PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.LEFT));
         zedLeftColorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
         addVisualizer(zedLeftColorImageVisualizer);
      }

      // ZED 2 color right image visualizer
      {
         zedRightColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Right",
                                                                          ros2Node,
                                                                          PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.RIGHT));
         zedRightColorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
         addVisualizer(zedRightColorImageVisualizer);
      }

      // ZED 2 depth image visualizer
      {
         zedDepthImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Depth Image", ros2Node, PerceptionAPI.ZED_DEPTH);
         zedDepthImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
         addVisualizer(zedDepthImageVisualizer);
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
         yoloVisualizer = new RDXROS2YOLOv8Visualizer("YOLOv8", ros2Node, ros2PeerClockOffsetEstimator, PerceptionAPI.YOLO_ANNOTATED_IMAGE);
         yoloVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_YOLO_ANNOTATED_IMAGE);
         addVisualizer(yoloVisualizer);
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
}

