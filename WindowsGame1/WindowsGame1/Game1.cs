using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;
using System.Threading;
using System.Threading.Tasks;
using System.Drawing;
using System.Drawing.Imaging;
using System.Diagnostics;


namespace KADA
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    /// 
    public struct DepthColor
    {
        public Vector3 Color;
        public int Depth;
        public bool UpToDate;

        public DepthColor(int Depth, Vector3 Color)
        {
            this.Depth = Depth;
            this.Color = Color;
            this.UpToDate = true;
        }
    }

    public class PCViewer : Microsoft.Xna.Framework.Game
    {
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;

        VertexBuffer instanceBuffer;
        VertexBuffer geometryBuffer;
        IndexBuffer indexBuffer;

        VertexBufferBinding[] bindings;

        VertexDeclaration instanceVertexDeclaration;

        
        Effect effect;
        private Vector3 CameraPosition = new Vector3(0, 0, -200);
        private Vector3 CameraLookAt = new Vector3(0, 0, -400);

        private Matrix World;
        private Matrix View;
        private Matrix Projection;

        private Queue<DepthColor[,]> depths, depthsPool;
        SpriteFont spriteFont;
        int totalFrames = 0;
        int elapsedTime = 0;
        float fps = 0;
        Random rnd = new Random();
       
        Task transformationUpdater;
        //Task cleanUpBufferTask;

        struct InstanceInfo
        {
            public Vector3 ScreenPos;
            public float Scale;
            public Vector3 Color;
        };

        private void GenerateInstanceVertexDeclaration()
        {
            VertexElement[] instanceStreamElements = new VertexElement[3];
            instanceStreamElements[0] = new VertexElement(0, VertexElementFormat.Vector3, VertexElementUsage.TextureCoordinate, 1);
            instanceStreamElements[1] = new VertexElement(sizeof(float) * 3, VertexElementFormat.Single, VertexElementUsage.TextureCoordinate, 2);
            instanceStreamElements[2] = new VertexElement(sizeof(float) * 4, VertexElementFormat.Vector3, VertexElementUsage.Color, 0);
            instanceVertexDeclaration = new VertexDeclaration(instanceStreamElements);
        }

        Int32 count = 640 * 480;
        InstanceInfo[] instances;

        private void GenerateInstanceInformation()
        {
            instances = new InstanceInfo[count];
            Random rnd = new Random();
            for (int i = 0; i < count; i++)
            {
                instances[i].ScreenPos =
                new Vector3(0,0,0);
                instances[i].Scale = 1;
                instances[i].Color = new Vector3(0,0,0);
            }
            instanceBuffer = new VertexBuffer(GraphicsDevice, instanceVertexDeclaration, count, BufferUsage.WriteOnly);
            instanceBuffer.SetData(instances);
        }

        private void UpdateInstanceInformation()
        {
            
            
            DepthColor[,] depth = null;
            bool frameLoaded = false;
            lock (this.depths)
            {                
                if (this.depths.Count > 1)
                {
                    depth = this.depths.Dequeue();
                    frameLoaded = true;
                }
            }

            if (frameLoaded)
            {
                int i = 0;
                for (int x = 0; x < depth.GetLength(0); x++)
                {
                    for (int y = 0; y < depth.GetLength(1); y++)
                    {
                        DepthColor d = depth[x, y];
                        if (d.UpToDate)
                        {
                            //instances[i].ScreenPos =
                            //new Vector3(x - 320, -(y - 240),-1000); //-d.Depth
                            Vector3 pos = instances[i].ScreenPos;
                            pos.X = x - 320;
                            pos.Y = -(y - 240);
                            pos.Z = -1000;
                            instances[i].ScreenPos = pos;
                            instances[i].Scale = -d.Depth;//(float)Math.Sqrt(Math.Pow(d.Depth,2)+Math.Pow(pos.X,2))/1000f;
                            instances[i].Color = d.Color;
                        }
                        else
                        {
                            instances[i].ScreenPos.Z = float.MaxValue;
                            instances[i].Scale = 1;
                        }
                        i++;
                    }

                }
                lock (GraphicsDevice)
                {
                    GraphicsDevice.SetVertexBuffers(null);
                    instanceBuffer.SetData(instances);
                }


                lock (this.depthsPool)
                {
                    this.depthsPool.Enqueue(depth);
                }
                //cleanUpBufferTask = new Task(() => this.CleanUpBuffer(depth));
                //cleanUpBufferTask.Start();
                
            }
            

        }
        private void CleanUpBuffer(DepthColor[,] depth)
        {
           /* for (int x = 0; x < depth.GetLength(0); x++)
            {
                for (int y = 0; y < depth.GetLength(1); y++)
                {
                    depth[x, y].UpToDate = false;
                }

            }*/

            lock (this.depthsPool)
            {
                this.depthsPool.Enqueue(depth);
            }
            
        }

        public PCViewer(Queue<DepthColor[,]> depths, Queue<DepthColor[,]> depthsPool)
        {

            
            this.depths = depths;
            this.depthsPool = depthsPool;
            graphics = new GraphicsDeviceManager(this);
            graphics.IsFullScreen = false;
            graphics.PreferredBackBufferWidth = 640;
            graphics.PreferredBackBufferHeight = 480;


            graphics.ApplyChanges();

            Content.RootDirectory = "Content";
        }



        /// <summary>
        /// Allows the game to perform any initialization it needs to before starting to run.
        /// This is where it can query for any required services and load any non-graphic
        /// related content.  Calling base.Initialize will enumerate through any components
        /// and initialize them as well.
        /// </summary>
        protected override void Initialize()
        {
            // TODO: Add your initialization logic here
            World = Matrix.Identity;
            UpdateView();
            Projection = Matrix.CreatePerspectiveFieldOfView(MathHelper.PiOver4, GraphicsDevice.Viewport.AspectRatio, 1, 100000);
            base.Initialize();
        }

        protected void UpdateView()
        {
            View = Matrix.CreateLookAt(CameraPosition, CameraLookAt, Vector3.Up);
        }

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input, and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {
            // Allows the game to exit
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed)
                this.Exit();

            elapsedTime += (int)gameTime.ElapsedGameTime.TotalMilliseconds;
            if (elapsedTime > 1000)
            {
                fps = totalFrames;
                totalFrames = 0;
                elapsedTime = 0;
            }

            // TODO: Add your update logic here
            transformationUpdater = new Task(() => this.UpdateInstanceInformation());
            transformationUpdater.Start();

            HandleInput(Keyboard.GetState());

            base.Update(gameTime);
        }

        protected void HandleInput(KeyboardState kS)
        {
            if (kS.IsKeyDown(Keys.A))
            {
                Matrix rotY = new Matrix();
                rotY = Matrix.CreateRotationY(0.02f);
                Vector3 CameraLookAtNew = CameraLookAt - CameraPosition;
                CameraLookAtNew = Vector3.Transform(CameraLookAtNew, rotY);
                CameraLookAtNew += CameraPosition;
                CameraLookAt = CameraLookAtNew;

                

            }
            if (kS.IsKeyDown(Keys.D))
            {
                Matrix rotY = new Matrix();
                rotY = Matrix.CreateRotationY(-0.02f);
                Vector3 CameraLookAtNew = CameraLookAt - CameraPosition;
                CameraLookAtNew = Vector3.Transform(CameraLookAtNew, rotY);
                CameraLookAtNew += CameraPosition;
                CameraLookAt = CameraLookAtNew;

                

            }
            if(kS.IsKeyDown(Keys.W)){
                Vector3 direction = CameraLookAt-CameraPosition;
                direction.Normalize();
                direction *= 10;
                CameraPosition += direction;
                CameraLookAt += direction;
            }
            if (kS.IsKeyDown(Keys.S))
            {
                Vector3 direction = CameraLookAt - CameraPosition;
                direction.Normalize();
                direction *= 10;
                CameraPosition -= direction;
                CameraLookAt -= direction;
            }

            UpdateView();
        }



        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            // Create a new SpriteBatch, which can be used to draw textures.
            spriteBatch = new SpriteBatch(GraphicsDevice);
            GenerateGeometryBuffers();
            GenerateInstanceVertexDeclaration();
            GenerateInstanceInformation();
            spriteFont = Content.Load<SpriteFont>("FPS");

            bindings = new VertexBufferBinding[2];
            bindings[0] = new VertexBufferBinding(geometryBuffer);
            bindings[1] = new VertexBufferBinding(instanceBuffer, 0, 1);

            effect = Content.Load<Effect>("InstancingEffect");


            effect.CurrentTechnique = effect.Techniques["Instancing"];
            effect.Parameters["WVP"].SetValue(View * Projection);


            // TODO: use this.Content to load your game content here
        }

        private void GenerateGeometryBuffers()
        {
            VertexPositionTexture[] vertices = new VertexPositionTexture[24];
            vertices[0].Position = new Vector3(-1, 1, 0);
            vertices[0].TextureCoordinate = new Vector2(0, 0);
            vertices[1].Position = new Vector3(1, 1, 0);
            vertices[1].TextureCoordinate = new Vector2(1, 0);
            vertices[2].Position = new Vector3(-1, 1, 0.1f);
            vertices[2].TextureCoordinate = new Vector2(0, 1);
            vertices[3].Position = new Vector3(1, 1, 0.1f);
            vertices[3].TextureCoordinate = new Vector2(1, 1);

            vertices[4].Position = new Vector3(-1, -1, 0.1f);
            vertices[4].TextureCoordinate = new Vector2(0, 0);
            vertices[5].Position = new Vector3(1, -1, 0.1f);
            vertices[5].TextureCoordinate = new Vector2(1, 0);
            vertices[6].Position = new Vector3(-1, -1, 0);
            vertices[6].TextureCoordinate = new Vector2(0, 1);
            vertices[7].Position = new Vector3(1, -1, 0);
            vertices[7].TextureCoordinate = new Vector2(1, 1);

            vertices[8].Position = new Vector3(-1, 1, 0);
            vertices[8].TextureCoordinate = new Vector2(0, 0);
            vertices[9].Position = new Vector3(-1, 1, 0.1f);
            vertices[9].TextureCoordinate = new Vector2(1, 0);
            vertices[10].Position = new Vector3(-1, -1, 0);
            vertices[10].TextureCoordinate = new Vector2(0, 1);
            vertices[11].Position = new Vector3(-1, -1, 0.1f);
            vertices[11].TextureCoordinate = new Vector2(1, 1);

            vertices[12].Position = new Vector3(-1, 1, 0.1f);
            vertices[12].TextureCoordinate = new Vector2(0, 0);
            vertices[13].Position = new Vector3(1, 1, 0.1f);
            vertices[13].TextureCoordinate = new Vector2(1, 0);
            vertices[14].Position = new Vector3(-1, -1, 0.1f);
            vertices[14].TextureCoordinate = new Vector2(0, 1);
            vertices[15].Position = new Vector3(1, -1, 0.1f);
            vertices[15].TextureCoordinate = new Vector2(1, 1);

            vertices[16].Position = new Vector3(1, 1, 0.1f);
            vertices[16].TextureCoordinate = new Vector2(0, 0);
            vertices[17].Position = new Vector3(1, 1, 0);
            vertices[17].TextureCoordinate = new Vector2(1, 0);
            vertices[18].Position = new Vector3(1, -1, 0.1f);
            vertices[18].TextureCoordinate = new Vector2(0, 1);
            vertices[19].Position = new Vector3(1, -1, 0);
            vertices[19].TextureCoordinate = new Vector2(1, 1);

            vertices[20].Position = new Vector3(1, 1, 0);
            vertices[20].TextureCoordinate = new Vector2(0, 0);
            vertices[21].Position = new Vector3(-1, 1, 0);
            vertices[21].TextureCoordinate = new Vector2(1, 0);
            vertices[22].Position = new Vector3(1, -1, 0);
            vertices[22].TextureCoordinate = new Vector2(0, 1);
            vertices[23].Position = new Vector3(-1, -1, 0);
            vertices[23].TextureCoordinate = new Vector2(1, 1);

            geometryBuffer = new VertexBuffer(GraphicsDevice, VertexPositionTexture.VertexDeclaration, 24, BufferUsage.WriteOnly);
            geometryBuffer.SetData(vertices);

            int[] indices = new int[36];
            indices[0] = 0; indices[1] = 1; indices[2] = 2;
            indices[3] = 1; indices[4] = 3; indices[5] = 2;

            indices[6] = 4; indices[7] = 5; indices[8] = 6;
            indices[9] = 5; indices[10] = 7; indices[11] = 6;

            indices[12] = 8; indices[13] = 9; indices[14] = 10;
            indices[15] = 9; indices[16] = 11; indices[17] = 10;

            indices[18] = 12; indices[19] = 13; indices[20] = 14;
            indices[21] = 13; indices[22] = 15; indices[23] = 14;

            indices[24] = 16; indices[25] = 17; indices[26] = 18;
            indices[27] = 17; indices[28] = 19; indices[29] = 18;

            indices[30] = 20; indices[31] = 21; indices[32] = 22;
            indices[33] = 21; indices[34] = 23; indices[35] = 22;

            indexBuffer = new IndexBuffer(GraphicsDevice, typeof(int), 36, BufferUsage.WriteOnly);
            indexBuffer.SetData(indices);
        }

        /// <summary>
        /// UnloadContent will be called once per game and is the place to unload
        /// all content.
        /// </summary>
        protected override void UnloadContent()
        {
            // TODO: Unload any non ContentManager content here
        }


        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Microsoft.Xna.Framework.Color.CornflowerBlue);
            
            effect.CurrentTechnique = effect.Techniques["Instancing"];
            effect.Parameters["WVP"].SetValue(View * Projection);
            lock (GraphicsDevice)
            {
                GraphicsDevice.Indices = indexBuffer;

                effect.CurrentTechnique.Passes[0].Apply();

                GraphicsDevice.SetVertexBuffers(bindings);

                GraphicsDevice.DrawInstancedPrimitives(PrimitiveType.TriangleList, 0, 0, 24, 0, 12, count);
                
                /*spriteBatch.Begin();
                spriteBatch.DrawString(spriteFont, string.Format("FPS={0}", fps),
                    new Vector2(10.0f, 20.0f),Microsoft.Xna.Framework.Color.Green);
                spriteBatch.End();
                totalFrames++;*/
            }

            

            base.Draw(gameTime);

        }




    }
}
