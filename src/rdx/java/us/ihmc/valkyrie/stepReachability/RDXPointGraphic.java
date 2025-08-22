package us.ihmc.valkyrie.stepReachability;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class RDXPointGraphic implements RenderableProvider
{
   private final ModelBuilder modelBuilder = new ModelBuilder();
   RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();

   private volatile Runnable toRender = null;
   private ModelInstance modelInstance;
   private Model lastModel;
   private double pointSize = 0.03;
   private Color color = Color.BLUE;

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   public void render()
   {
      if (toRender != null)
      {
         toRender.run();
         toRender = null;
      }
   }

   public void generateMeshesAsync(us.ihmc.euclid.tuple3D.Point3D point)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(point));
   }

   public synchronized void generateMeshes(us.ihmc.euclid.tuple3D.Point3D point)
   {
      meshBuilder.clear();
      meshBuilder.addSphere((float) pointSize, point, color);
      toRender = () ->
      {
         modelBuilder.begin();
         Mesh mesh = meshBuilder.generateMesh();
         MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
         Material material = new Material();
         Texture paletteTexture = new Texture(Gdx.files.classpath("palette.png"));
         material.set(TextureAttribute.createDiffuse(paletteTexture));
         float shade = 0.6f;
         material.set(ColorAttribute.createDiffuse(shade, shade, shade, 1.0f));
         modelBuilder.part(meshPart, material);

         if (lastModel != null)
            lastModel.dispose();

         lastModel = modelBuilder.end();
         modelInstance = new ModelInstance(lastModel); // TODO: Clean up garbage and look into reusing the Model
      };
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }

   public void setColor(Color color)
   {
      this.color = color;
   }

   public void destroy()
   {
      executorService.destroy();
   }
}
