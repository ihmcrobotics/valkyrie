package us.ihmc.valkyrie.stepReachability;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.util.*;
import java.util.stream.Collectors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.multiContact.CenterOfMassMotionControlAnchorDescription;
import us.ihmc.avatar.multiContact.KinematicsToolboxSnapshotDescription;
import us.ihmc.avatar.multiContact.MultiContactScriptReader;
import us.ihmc.avatar.multiContact.SixDoFMotionControlAnchorDescription;
import us.ihmc.avatar.reachabilityMap.footstep.StepReachabilityIOHelper;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityLatticePoint;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.rdx.BufferBasedColorProvider;
import us.ihmc.rdx.RDXPointCloudRendererOld;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.messager.Messager;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspacePathTools;

public class RDXStepReachabilityPanel implements RDXRenderableProvider
{
   private final String windowName = ImGuiTools.uniqueLabel(this, "Step Reachability");
   private DRCRobotModel robotModel;
   private RDXMultiBodyGraphic ikRobot;
   private FullHumanoidRobotModel ikFullRobotModel;
   private RDXFootstepPlanGraphic footstepPlanGraphic;
   private RDXPointGraphic chestObjective;
   private RDXPointGraphic headObjective;
   private RDXPointGraphic chestPose;
   private RDXPointGraphic headPose;
   private RDXPointGraphic comObjective;
   private RDXPointGraphic comPose;
   private RDXPointCloudRendererOld reachabilityPointCloud;
   private ImBoolean pointCloudCreated = new ImBoolean(false);

   private ImBoolean showPointCloud = new ImBoolean(true);
   private ImBoolean showFootsteps = new ImBoolean(true);
   private ImBoolean showChestObjective = new ImBoolean(true);
   private ImBoolean showHeadObjective = new ImBoolean(true);
   private ImBoolean showCoMObjective = new ImBoolean(true);
   private ImBoolean showFeasibleSteps = new ImBoolean(true);
   private ImBoolean showInfeasibleSteps = new ImBoolean(true);
   private ImBoolean showFeasibleStepsInPointCloud = new ImBoolean(true);
   private ImBoolean showInfeasibleStepsInPointCloud = new ImBoolean(true);
   private ImBoolean preserveX = new ImBoolean(true);
   private ImBoolean preserveY = new ImBoolean(true);
   private ImBoolean preserveZ = new ImBoolean(true);
   private ImBoolean preserveYaw = new ImBoolean(true);
   private boolean suppressPointCloud = false;

   private int selectedScript = -1;
   private int selectedStep = -1;
   private StepReachabilityHelper helper;
   private ImInt selectedKeyframe = new ImInt();
   private final ArrayList<Path> scripts = new ArrayList<>();
   private final MultiContactScriptReader scriptReader = new MultiContactScriptReader();
   private final List<KinematicsToolboxSnapshotDescription> loadedScript = scriptReader.getAllItems();
   private final List<KinematicsToolboxSnapshotDescription> editedScript = new ArrayList<>();
   private boolean loadedDatasetsOnce = false;

   private StepReachabilityData stepReachabilityData;
   private double yawSpacing;
   private ImFloat reachabilityThreshold = new ImFloat();
   private ImDouble footX = new ImDouble();
   private ImDouble footY = new ImDouble();
   private ImDouble footZ = new ImDouble();
   private ImDouble footYaw = new ImDouble();

   private static final Timer timer = new Timer();
   ImBoolean showNoStepFoundMessage = new ImBoolean(false);

   public void create(DRCRobotModel robotModel, Messager messager)
   {
      helper = new StepReachabilityHelper(messager);
      this.robotModel = robotModel;
      stepReachabilityData = robotModel.getStepReachabilityData();
      yawSpacing = stepReachabilityData.getGridSizeYaw() / stepReachabilityData.getYawDivisions();
      reachabilityPointCloud = new RDXPointCloudRendererOld();
      footstepPlanGraphic = new RDXFootstepPlanGraphic();
      chestObjective = new RDXPointGraphic();
      chestPose = new RDXPointGraphic();
      chestPose.setColor(Color.RED);
      headObjective = new RDXPointGraphic();
      headPose = new RDXPointGraphic();
      headPose.setColor(Color.RED);
      comObjective = new RDXPointGraphic();
      comPose = new RDXPointGraphic();
      comPose.setColor(Color.RED);

      ikFullRobotModel = robotModel.createFullRobotModel();
      OneDoFJointBasics[] ikJoints = FullRobotModelUtils.getAllJointsExcludingHands(ikFullRobotModel);
      ikRobot = new RDXMultiBodyGraphic(robotModel.getSimpleRobotName());
      ikRobot.loadRobotModelAndGraphics(robotModel.getRobotDefinition(), ikFullRobotModel.getElevator());

      helper.subscribeToIKOutput(output ->
                                 {
                                    synchronized (ikFullRobotModel)
                                    {
                                       ikFullRobotModel.getRootJoint().setJointConfiguration(output.getMiddle(), output.getLeft());

                                       for (int i = 0; i < ikJoints.length; i++)
                                          ikJoints[i].setQ(output.getRight()[i]);

                                       ikFullRobotModel.getElevator().updateFramesRecursively();
                                       ikFullRobotModel.getChest().updateFramesRecursively();
                                       ikFullRobotModel.getHead().updateFramesRecursively();
                                    }
                                 });
   }

   public void render()
   {
      synchronized (ikFullRobotModel)
      {
         ikRobot.update();
      }
      if (showFootsteps.get())
         footstepPlanGraphic.update();

      if (showChestObjective.get())
      {
         chestObjective.render();
         chestPose.render();
      }
      if (showHeadObjective.get())
      {
         headObjective.render();
         headPose.render();
      }
      if (showCoMObjective.get())
      {
         comObjective.render();
         comPose.render();
      }

      ImGui.begin(ImGuiTools.uniqueLabel(this, "Scripts"));
      if (!loadedDatasetsOnce)
      {
         loadedDatasetsOnce = true;
         Path scriptsPath = WorkspacePathTools.findPathToResource("ihmc-open-robotics-software/valkyrie/src/main/resources", "us/ihmc/valkyrie/", "parameters");
         scripts.clear();
         PathTools.walkFlat(scriptsPath, (path, pathType) ->
         {
            if (pathType == BasicPathVisitor.PathType.FILE)
            {
               scripts.add(path);
            }
            return FileVisitResult.CONTINUE;
         });
         if (!pointCloudCreated.get())
         {
            reachabilityPointCloud.create(findMaxScriptSize());
            pointCloudCreated.set(true);
         }
      }

      // Script selection
      for (int i = 0; i < scripts.size(); i++)
      {
         if (ImGui.radioButton(scripts.get(i).getFileName().toString(), selectedScript == i))
         {
            boolean success = scriptReader.loadScript(scripts.get(i).toFile());
            if (success)
            {
               selectedScript = i;
               // load script
               editedScript.clear();
               for (int j = 0; j < loadedScript.size(); j++)
               {
                  editedScript.add(new KinematicsToolboxSnapshotDescription(loadedScript.get(j)));
               }
               StepReachabilityIOHelper stepReachabilityIOHelper = new StepReachabilityIOHelper();
               stepReachabilityData = stepReachabilityIOHelper.loadStepReachability(robotModel);
               yawSpacing = stepReachabilityData.getGridSizeYaw() / stepReachabilityData.getYawDivisions();

               selectedKeyframe.set(0);
               loadNewFrame();
            }
         }
      }
      ImGui.end();

      // Step selection in selected script
      ImGui.begin("Steps in script");
      for (int i = 0; i < editedScript.size(); i++)
      {
         StepReachabilityLatticePoint latticePoint = getLatticePointFromScriptIndex(i, editedScript);
         if (!showFeasibleSteps.get() && stepReachabilityData.getLegReachabilityMap().get(latticePoint) < reachabilityThreshold.get())
            continue;
         if (!showInfeasibleSteps.get() && stepReachabilityData.getLegReachabilityMap().get(latticePoint) > reachabilityThreshold.get())
            continue;
         if (ImGui.radioButton(latticePoint.toString(), selectedStep == i))
         {
            selectedStep = i;
            selectedKeyframe.set(i);

            footX.set(latticePoint.getXIndex() * stepReachabilityData.getXyzSpacing());
            footY.set(latticePoint.getYIndex() * stepReachabilityData.getXyzSpacing());
            footZ.set(latticePoint.getZIndex() * stepReachabilityData.getXyzSpacing());
            footYaw.set(latticePoint.getYawIndex() * yawSpacing);

            loadNewFrame();
         }
      }
      ImGui.end();

      ImGui.begin("Step Reachability");

      // Frame selection for selected script
      ImGui.text("Frame selection");
      ImGui.text("Number of keyframes: " + editedScript.size());
      ImGui.pushItemWidth(100.0f);

      boolean keyFrameSelected = false;
      keyFrameSelected |= ImGui.dragInt(ImGuiTools.uniqueLabel(this, "Keyframe"), selectedKeyframe.getData(), 0.1f, 0, editedScript.size());
      if (keyFrameSelected)
      {
         if (selectedKeyframe.get() > editedScript.size() - 1)
         {
            selectedKeyframe.set(editedScript.size() - 1);
            selectedStep = editedScript.size() - 1;
         }
         else if (selectedKeyframe.get() < 0)
         {
            selectedKeyframe.set(0);
            selectedStep = 0;
         }
         else
         {
            selectedStep = selectedKeyframe.get();
            loadNewFrame();
         }
      }
      ImGui.popItemWidth();

      ImGui.newLine();
      ImGui.newLine();

      // Get input for X,Y,Z, and Yaw
      ImGui.text("Step selection");
      ImGui.text("Solve robot configuration for step:");
      ImGui.pushItemWidth(100.0f);
      ImGui.inputDouble(ImGuiTools.uniqueLabel(this, "X"), footX);
      ImGui.inputDouble(ImGuiTools.uniqueLabel(this, "Y"), footY);
      ImGui.inputDouble(ImGuiTools.uniqueLabel(this, "Z"), footZ);
      ImGui.inputDouble(ImGuiTools.uniqueLabel(this, "Yaw"), footYaw);

      int scriptIndexOfStep = getScriptIndexOfStep();
      if (scriptIndexOfStep == -1)
      {
         ImGui.text("Step is outside of feasibility map");
      }
      else
      {
         ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Set reachability threshold"), reachabilityThreshold.getData(), 0.03f, 0, 50);

         selectedStep = scriptIndexOfStep;
         selectedKeyframe.set(scriptIndexOfStep);
         double solutionQuality = editedScript.get(selectedKeyframe.get()).getIkSolution().getSolutionQuality();
         ImGui.text("Solution quality: " + solutionQuality);
         if (solutionQuality > reachabilityThreshold.get())
         {
            boolean getClosestFeasibleStepButtonClicked = (imgui.internal.ImGui.button(ImGuiTools.uniqueLabel(this, "Snap to closest feasible step")));
            ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Preserve X axis"), preserveX);
            ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Preserve Y axis"), preserveY);
            ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Preserve Z axis"), preserveZ);
            ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Preserve Yaw"), preserveYaw);
            if (getClosestFeasibleStepButtonClicked)
            {
               snapToClosestFeasibleStep();
            }
         }
         loadNewFrame();
      }
      if (showNoStepFoundMessage.get())
         ImGui.text("No feasible step found");

      ImGui.end();

      ImGui.begin("View Options");

      ImGui.text("Script");
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Show feasible steps in script"), showFeasibleSteps);
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Show infeasible steps in script"), showInfeasibleSteps);

      ImGui.newLine();
      ImGui.text("Objectives");
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Show foot objectives"), showFootsteps);
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Show chest objective"), showChestObjective);
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Show head objective"), showHeadObjective);
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Show center of mass objective"), showCoMObjective);
      ImGui.text("Desired in blue, actual in red");

      ImGui.newLine();
      ImGui.text("Point Cloud");
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Show feasible steps"), showFeasibleStepsInPointCloud);
      ImGui.checkbox(ImGuiTools.uniqueLabel(this, "Show infeasible steps"), showInfeasibleStepsInPointCloud);

      ImGui.end();
   }

   private void loadNewFrame()
   {
      KinematicsToolboxSnapshotDescription kinematicsToolboxSnapshotDescription = editedScript.get(selectedKeyframe.get());
      KinematicsToolboxOutputStatus ikSolution = kinematicsToolboxSnapshotDescription.getIkSolution();
      helper.publish(StepReachabilityMessagerAPI.IKOutput, ikSolution);

      if (showFootsteps.get())
         loadFootstep();

      if (showChestObjective.get())
         loadChestObjective();

      if (showHeadObjective.get())
         loadHeadObjective();

      if (showCoMObjective.get())
         loadCoMObjective();

      if (showPointCloud.get() && reachabilityPointCloud != null)
         updatePointCloud();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      ikRobot.getRenderables(renderables, pool, sceneLevels);

      if (showFootsteps.get())
         footstepPlanGraphic.getRenderables(renderables, pool);

      if (showChestObjective.get())
      {
         chestObjective.getRenderables(renderables, pool);
         chestPose.getRenderables(renderables, pool);
      }

      if (showHeadObjective.get())
      {
         headObjective.getRenderables(renderables, pool);
         headPose.getRenderables(renderables, pool);
      }

      if (showCoMObjective.get())
      {
         comObjective.getRenderables(renderables, pool);
         comPose.getRenderables(renderables, pool);
      }

      if (showPointCloud.get() && reachabilityPointCloud != null && !suppressPointCloud)
         reachabilityPointCloud.getRenderables(renderables, pool);
   }

   public String getWindowName()
   {
      return windowName;
   }

   public void dispose()
   {
      reachabilityPointCloud.dispose();
      footstepPlanGraphic.destroy();
      chestObjective.destroy();
      headObjective.destroy();
      chestPose.destroy();
      headPose.destroy();
      comObjective.destroy();
      comPose.destroy();
   }

   public void snapToClosestFeasibleStep()
   {
      StepReachabilityLatticePoint inputStep = new StepReachabilityLatticePoint(footX.get(),
                                                                                footY.get(),
                                                                                footZ.get(),
                                                                                footYaw.get(),
                                                                                stepReachabilityData.getXyzSpacing(),
                                                                                stepReachabilityData.getYawDivisions(),
                                                                                yawSpacing);
      Map<StepReachabilityLatticePoint, Double> legReachabilityMap = stepReachabilityData.getLegReachabilityMap();
      double closestDist = 100;
      StepReachabilityLatticePoint nearestFeasibleStep = new StepReachabilityLatticePoint(0, 0, 0, 0);
      for (StepReachabilityLatticePoint latticePoint : legReachabilityMap.keySet())
      {
         double currentDist = latticePoint.indexDistanceFrom(inputStep);
         if (currentDist < closestDist && legReachabilityMap.get(latticePoint) < reachabilityThreshold.get())
         {
            // If step is not along preserved axes, doesn't count
            boolean pass = true;
            boolean sameX = latticePoint.getXIndex() == inputStep.getXIndex();
            boolean sameY = latticePoint.getYIndex() == inputStep.getYIndex();
            boolean sameZ = latticePoint.getZIndex() == inputStep.getZIndex();
            boolean sameYaw = latticePoint.getYawIndex() == inputStep.getYawIndex();
            if ((preserveX.get() && !sameX) || (preserveY.get() && !sameY) || (preserveZ.get() && !sameZ) || (preserveYaw.get() && !sameYaw))
               pass = false;

            if (pass == true)
            {
               closestDist = currentDist;
               nearestFeasibleStep = latticePoint;
            }
         }
      }
      // If reachable step is found (i.e. if nearestFeasibleStep is changed)
      if (!nearestFeasibleStep.equals(new StepReachabilityLatticePoint(0, 0, 0, 0)))
      {
         footX.set(nearestFeasibleStep.getXIndex() * stepReachabilityData.getXyzSpacing());
         footY.set(nearestFeasibleStep.getYIndex() * stepReachabilityData.getXyzSpacing());
         footZ.set(nearestFeasibleStep.getZIndex() * stepReachabilityData.getXyzSpacing());
         footYaw.set(nearestFeasibleStep.getYawIndex() * yawSpacing);
         selectedKeyframe.set(getScriptIndexOfStep());
      }
      else
      {
         showNoStepFoundMessage.set(true);
         timer.schedule(new TimerTask()
         {
            @Override
            public void run()
            {
               showNoStepFoundMessage.set(false);
            }
         }, 800);
      }
   }

   private void updatePointCloud()
   {
      BufferBasedColorProvider colorProvider = new BufferBasedColorProvider();
      RecyclingArrayList<Point3D32> stepPoints = new RecyclingArrayList<>(Point3D32::new);

      List<KinematicsToolboxSnapshotDescription> script = new ArrayList<>();
      if (showFeasibleStepsInPointCloud.get())
         script.addAll(editedScript.stream()
                                   .filter(snapshot -> snapshot.getIkSolution().getSolutionQuality()
                                                       < reachabilityThreshold.get())
                                   .collect(Collectors.toList()));
      if (showInfeasibleStepsInPointCloud.get())
         script.addAll(editedScript.stream()
                                   .filter(snapshot -> snapshot.getIkSolution().getSolutionQuality()
                                                       > reachabilityThreshold.get())
                                   .collect(Collectors.toList()));

      if (script.isEmpty())
         suppressPointCloud = true;
      else
         suppressPointCloud = false;

      HashMap<Point3D32, Double> yawAveragedReachabilityMap = new HashMap<>();
      for (int i = 0; i < script.size(); i++)
      {
         StepReachabilityLatticePoint footstep = getLatticePointFromScriptIndex(i, script);
         double reachabilityVal = stepReachabilityData.getLegReachabilityMap().get(footstep);
         Point3D32 footstepAsPoint = new Point3D32((float) (footstep.getXIndex() * stepReachabilityData.getXyzSpacing()),
                                                   (float) (footstep.getYIndex() * stepReachabilityData.getXyzSpacing()),
                                                   (float) (footstep.getZIndex() * stepReachabilityData.getXyzSpacing()));

         if (!yawAveragedReachabilityMap.containsKey(footstepAsPoint))
         {
            yawAveragedReachabilityMap.put(footstepAsPoint, reachabilityVal);
         }
         else
         {
            double updatedReachabilityVal = yawAveragedReachabilityMap.get(footstepAsPoint) + reachabilityVal;
            yawAveragedReachabilityMap.replace(footstepAsPoint, updatedReachabilityVal);
         }
      }

      for (Point3D32 step : yawAveragedReachabilityMap.keySet())
      {
         double averagedReachabilityVal = yawAveragedReachabilityMap.get(step) / stepReachabilityData.getYawDivisions();
         Point3D32 footstepAsPoint = stepPoints.add();
         footstepAsPoint.set(step);

         /* Color based on how valid above/below threshold
          * Yellow at threshold
          * The lower the value is below the threshold, the better/greener the solution
          * The higher the value is below the threshold, the worse/redder the solution
          */
         float r = 255;
         float g = 255;
         double difference = averagedReachabilityVal - reachabilityThreshold.get();
         if (difference <= 0)
            r = (float) (r + 20 * difference);
         else
            g = (float) (g - 20 * difference);
         colorProvider.add(new Color(r / 255, g / 255, 0.0f, 1.0f));
      }

      reachabilityPointCloud.setPointsToRender(stepPoints, colorProvider);
      reachabilityPointCloud.setPointScale(0.04f);
      reachabilityPointCloud.updateMesh(1.0f);
   }

   public int getScriptIndexOfStep()
   {
      StepReachabilityLatticePoint inputPoint = new StepReachabilityLatticePoint(footX.get(),
                                                                                 footY.get(),
                                                                                 footZ.get(),
                                                                                 footYaw.get(),
                                                                                 stepReachabilityData.getXyzSpacing(),
                                                                                 stepReachabilityData.getYawDivisions(),
                                                                                 yawSpacing);
      if (!stepReachabilityData.getLegReachabilityMap().containsKey(inputPoint))
      {
         return -1;
      }
      else
      {
         for (int i = 0; i < editedScript.size(); i++)
         {
            StepReachabilityLatticePoint scriptPoint = getLatticePointFromScriptIndex(i, editedScript);

            if (scriptPoint.equals(inputPoint))
               return i;
         }
      }
      return -1;
   }

   private StepReachabilityLatticePoint getLatticePointFromScriptIndex(int index, List<KinematicsToolboxSnapshotDescription> script)
   {
      KinematicsToolboxSnapshotDescription snapshot = script.get(index);
      SixDoFMotionControlAnchorDescription leftFoot = snapshot.getSixDoFAnchors().get(0);
      assert (leftFoot.getRigidBodyName().equals(ikFullRobotModel.getFoot(RobotSide.LEFT).getName()));

      Point3D leftFootDesiredPosition = leftFoot.getInputMessage().getDesiredPositionInWorld();
      Quaternion leftFootDesiredOrientation = leftFoot.getInputMessage().getDesiredOrientationInWorld();
      StepReachabilityLatticePoint latticePoint = new StepReachabilityLatticePoint(leftFootDesiredPosition.getX(),
                                                                                   leftFootDesiredPosition.getY(),
                                                                                   leftFootDesiredPosition.getZ(),
                                                                                   leftFootDesiredOrientation.getYaw(),
                                                                                   stepReachabilityData.getXyzSpacing(),
                                                                                   stepReachabilityData.getYawDivisions(),
                                                                                   yawSpacing);
      return latticePoint;
   }

   private void loadFootstep()
   {
      updateFootPoseFromFrame();
      ArrayList<MinimalFootstep> footsteps = new ArrayList<>();
      Pose3D leftPose = new Pose3D();
      leftPose.set(footX.get(), footY.get(), footZ.get(), footYaw.get(), 0, 0);
      ConvexPolygon2D foothold = new ConvexPolygon2D();
      double halfLength = 0.1;
      double halfWidth = 0.05;
      foothold.addVertex(halfLength, -halfWidth);
      foothold.addVertex(halfLength, halfWidth);
      foothold.addVertex(-halfLength, halfWidth);
      foothold.addVertex(-halfLength, -halfWidth);
      foothold.update();
      footsteps.add(new MinimalFootstep(RobotSide.LEFT, leftPose, foothold, "Reachability Left"));
      Pose3D rightPose = new Pose3D();
      rightPose.set(0, 0, 0, 0, 0, 0);
      footsteps.add(new MinimalFootstep(RobotSide.RIGHT, rightPose, foothold, "Reachability Right"));
      footstepPlanGraphic.generateMeshes(footsteps);
   }

   private void loadChestObjective()
   {
      RigidBodyBasics chestRigidBody = ikFullRobotModel.getChest();
      FramePose3D currentChestPose = new FramePose3D(chestRigidBody.getBodyFixedFrame());
      currentChestPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D actualChestPose = new Point3D(currentChestPose.getPosition());

      KinematicsToolboxSnapshotDescription snapshot = editedScript.get(selectedKeyframe.get());
      SixDoFMotionControlAnchorDescription chest = snapshot.getSixDoFAnchors().get(2);
      assert (chest.getRigidBodyName().equals(ikFullRobotModel.getChest()));
      Point3D desiredChestPose = chest.getInputMessage().getDesiredPositionInWorld();
      desiredChestPose.setZ(actualChestPose.getZ());

      chestObjective.generateMeshes(desiredChestPose);
      chestPose.generateMeshes(actualChestPose);
   }

   private void loadHeadObjective()
   {
      RigidBodyBasics headRigidBody = ikFullRobotModel.getHead();
      FramePose3D currentHeadPose = new FramePose3D(headRigidBody.getBodyFixedFrame());
      currentHeadPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D actualHeadPose = new Point3D(currentHeadPose.getPosition());

      KinematicsToolboxSnapshotDescription snapshot = editedScript.get(selectedKeyframe.get());
      SixDoFMotionControlAnchorDescription head = snapshot.getSixDoFAnchors().get(3);
      assert (head.getRigidBodyName().equals(ikFullRobotModel.getHead()));
      Point3D desiredHeadPose = head.getInputMessage().getDesiredPositionInWorld();
      desiredHeadPose.setZ(actualHeadPose.getZ());

      headObjective.generateMeshes(desiredHeadPose);
      headPose.generateMeshes(actualHeadPose);
   }

   private void loadCoMObjective()
   {
      RigidBodyBasics headRigidBody = ikFullRobotModel.getElevator();
      FramePose3D currentCoMPose = new FramePose3D(headRigidBody.getBodyFixedFrame());
      currentCoMPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D actualCoMPose = new Point3D(currentCoMPose.getPosition());

      KinematicsToolboxSnapshotDescription snapshot = editedScript.get(selectedKeyframe.get());
      CenterOfMassMotionControlAnchorDescription centerOfMass = snapshot.getCenterOfMassAnchor();
      Point3D desiredCoMPose = centerOfMass.getInputMessage().getDesiredPositionInWorld();

      comObjective.generateMeshes(desiredCoMPose);
      comPose.generateMeshes(actualCoMPose);
   }

   private int findMaxScriptSize()
   {
      int maxSize = 0;
      for (int i = 0; i < scripts.size(); i++)
      {
//         if (StepReachabilityFileTools.isReachabilityFile(scripts.get(i).toFile())) // FIXME
         {
            scriptReader.loadScript(scripts.get(i).toFile());
            int loadedScriptSize = loadedScript.size();
            if (loadedScriptSize > maxSize)
               maxSize = loadedScriptSize;
         }
      }
      LogTools.info(maxSize);
      return maxSize;
   }

   private void updateFootPoseFromFrame()
   {
      StepReachabilityLatticePoint footstep = getLatticePointFromScriptIndex(selectedKeyframe.get(), editedScript);
      footX.set(footstep.getXIndex() * stepReachabilityData.getXyzSpacing());
      footY.set(footstep.getYIndex() * stepReachabilityData.getXyzSpacing());
      footZ.set(footstep.getZIndex() * stepReachabilityData.getXyzSpacing());
      footYaw.set(footstep.getYawIndex() * yawSpacing);
   }
}