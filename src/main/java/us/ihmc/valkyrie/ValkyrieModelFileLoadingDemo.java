package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.parameterEstimation.InertiaVisualizationTools;
import us.ihmc.commonWalkingControlModules.parameterEstimation.YoInertiaEllipsoid;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaEllipsoidsVisualizer;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;

public class ValkyrieModelFileLoadingDemo
{
   private static final boolean SHOW_ELLIPSOIDS = false;
   private static final boolean SHOW_COORDINATES_AT_JOINT_ORIGIN = false;
   private static final boolean SHOW_INERTIA_ELLIPSOIDS = false;
   private static final boolean SHOW_KINEMATICS_COLLISIONS = false;
   private static final boolean SHOW_SIM_COLLISIONS = false;
   private static final boolean SHOW_HAND_CONTROL_FRAME = true;

   public ValkyrieModelFileLoadingDemo()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.ARM_MASS_SIM);

      if (SHOW_HAND_CONTROL_FRAME)
      {
         addHandControlFrames(robotModel.getRobotDefinition(), robotModel.getJointMap());
      }

      SimulationSession session = new SimulationSession();
      Robot simulationRobot = session.addRobot(robotModel.getRobotDefinition());

      YoGraphicGroupDefinition extraViz = new YoGraphicGroupDefinition("ExtraVisualization", new ArrayList<>());

      if (SHOW_INERTIA_ELLIPSOIDS)
      {
         ArrayList<YoInertiaEllipsoid> inertialEllipsoids = InertiaVisualizationTools.createYoInertiaEllipsoids(simulationRobot.getRootBody(),
                                                                                                                session.getRootRegistry());
         YoGraphicDefinition ellipsoidGroup = InertiaVisualizationTools.getInertiaEllipsoidGroup(inertialEllipsoids);
         extraViz.addChild(ellipsoidGroup);
      }

      session.addYoGraphicDefinitions(extraViz);
      SessionVisualizer.startSessionVisualizer(session);
   }

   private static void addHandControlFrames(RobotDefinition robotDefinition, ValkyrieJointMap jointMap)
   {
      double handControlFrameGraphicSize = 0.3;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform handControlFrameToWristTransform = jointMap.getHandControlFrameToWristTransform(robotSide);
         RigidBodyDefinition handDefinition = robotDefinition.getRigidBodyDefinition(jointMap.getHandName(robotSide));

         VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
         visualDefinitionFactory.appendTransform(handControlFrameToWristTransform);
         visualDefinitionFactory.addCoordinateSystem(handControlFrameGraphicSize);
         handDefinition.getVisualDefinitions().addAll(visualDefinitionFactory.getVisualDefinitions());
      }
   }

   public static void main(String[] args)
   {
      new ValkyrieModelFileLoadingDemo();
   }

}
