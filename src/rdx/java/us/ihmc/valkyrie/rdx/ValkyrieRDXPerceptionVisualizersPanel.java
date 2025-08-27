package us.ihmc.valkyrie.rdx;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXRobotPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXDetectionManagerSettings;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Visualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.zed.ZEDModelData;

/**
 * A common set of visualizers specific to sensors on Valkyrie
 */
public class ValkyrieRDXPerceptionVisualizersPanel extends RDXRobotPerceptionVisualizersPanel
{
   public ValkyrieRDXPerceptionVisualizersPanel(ROS2Node ros2Node,
                                                ROS2SyncedRobotModel syncedRobot,
                                                ROS2PeerClockOffsetEstimator ros2PeerClockOffsetEstimator)
   {
      super(ros2Node, syncedRobot, ros2PeerClockOffsetEstimator);

      // ZED colored point cloud visualizer
      {
         zedColoredPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED Mini Colored Point Cloud",
                                                                                 ros2Node,
                                                                                 PerceptionAPI.ZED_DEPTH,
                                                                                 PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.LEFT));
         zedColoredPointCloudVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
         zedColoredPointCloudVisualizer.setActive(true);
         addVisualizer(zedColoredPointCloudVisualizer);
      }

      // ZED left color visualizer
      {
         zedLeftColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED Mini Color Left",
                                                                         ros2Node,
                                                                         PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.LEFT));
         zedLeftColorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
         addVisualizer(zedLeftColorImageVisualizer);
      }

      // ZED color right image visualizer
      {
         zedRightColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED Mini Color Right",
                                                                          ros2Node,
                                                                          PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.RIGHT));
         zedRightColorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_PUBLICATION);
         addVisualizer(zedRightColorImageVisualizer);
      }

      // ZED depth image visualizer
      {
         zedDepthImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED Mini Depth Image", ros2Node, PerceptionAPI.ZED_DEPTH);
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

      // Detection Manager Settings visualizer
      {
         detectionManagerSettings = new RDXDetectionManagerSettings("Detection Manager Settings", ros2Node);
         addVisualizer(detectionManagerSettings);
      }
   }

   @Override
   public void setupAdditionalSensors(RDXBaseUI baseUI)
   {
      ValkyrieRDXROS2InteractableSensors interactableSensors = new ValkyrieRDXROS2InteractableSensors(baseUI,
                                                                                                      ros2Helper,
                                                                                                      syncedRobot,
                                                                                                      syncedRobot.getReferenceFrames(),
                                                                                                      robotVisualizer);
      interactableSensors.setupZEDMini();
   }

   @Override
   public ZEDModelData getZEDModelData()
   {
      return ZEDModelData.ZED_MINI;
   }
}

