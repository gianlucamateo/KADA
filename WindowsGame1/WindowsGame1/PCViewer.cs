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
using System.Collections.Concurrent;

using KADA;
using Point = KADA.Point;
using XNAModel = Microsoft.Xna.Framework.Graphics.Model;
using Model = KADA.Model;


namespace KADA
{
    #region DepthColor Struct

    public struct DepthColor
    {
        public Vector3 Color;
        public int Depth;
        public bool UpToDate;
        public Vector3 Position;
        public int BrickColorInteger;

        public DepthColor(int Depth, Vector3 Color)
        {
            this.Depth = Depth;
            this.Color = Color;
            this.UpToDate = true;
            this.Position = new Vector3(0, 0, 0);
            this.BrickColorInteger = 1;
        }
    }
    #endregion


    public class PCViewer : Microsoft.Xna.Framework.Game
    {
        #region Class Variables



        public Vector3[,] NormalMap = new Vector3[640, 480];

        

        Matrix RGBToYUV;
        Matrix YUVToRGB;
        Int32 count = 640 * 480 + 100000;
        Viewport PCViewport;
        Viewport BrickViewport;
        XNAModel brick, secondaryBrick;
        Texture2D brickTexture;

        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;

        VertexBuffer instanceBuffer;
        VertexBuffer geometryBuffer;
        IndexBuffer indexBuffer;

        VertexBufferBinding[] bindings;

        VertexDeclaration instanceVertexDeclaration;
        InstanceInfo[] instances;



        private Vector3 outlierCenter = Vector3.Zero;
        public Vector3 offset = new Vector3(-36.5f, -17f, 15f);

        private Matrix brickTranslation = Matrix.CreateTranslation(new Vector3(0, 0, 0));
        private Matrix brickRotation = Matrix.CreateTranslation(Vector3.Zero);




        Effect effect;
        private Vector3 CameraPosition = new Vector3(0, 0, 0);
        private Vector3 CameraLookAt = new Vector3(0, 0, -370);
        private Vector3 CameraUp;

        private bool track = false;

        public Matrix World;
        public Matrix View;
        public Matrix Projection;

        private ConcurrentQueue<DepthColor[,]> depths, depthsPool;
        SpriteFont spriteFont;
        int oldScroll;
        bool freeze = false;

        private Model model;
        private SortedDictionary<float, TentativeModel> tentativeModels;
        private PipelineManager manager;

        private PipelineDataContainer dataContainer;

        private List<Vector3> edgePositions = new List<Vector3>();

        Task transformationUpdater;

        #endregion

        struct InstanceInfo
        {
            public Vector3 ScreenPos;
            public float Scale;
            public Vector3 Color;
        };



        public void SetBrickTranslate(Matrix b)
        {
            this.brickTranslation = b;
        }

        public void SetBrickRotation(Matrix b)
        {
            this.brickRotation = b;
        }

        private void GenerateInstanceVertexDeclaration()
        {
            VertexElement[] instanceStreamElements = new VertexElement[3];
            instanceStreamElements[0] = new VertexElement(0, VertexElementFormat.Vector3, VertexElementUsage.TextureCoordinate, 1);
            instanceStreamElements[1] = new VertexElement(sizeof(float) * 3, VertexElementFormat.Single, VertexElementUsage.TextureCoordinate, 2);
            instanceStreamElements[2] = new VertexElement(sizeof(float) * 4, VertexElementFormat.Vector3, VertexElementUsage.Color, 0);
            instanceVertexDeclaration = new VertexDeclaration(instanceStreamElements);
        }

        private void GenerateInstanceInformation()
        {
            instances = new InstanceInfo[count];
            Random rnd = new Random();
            for (int i = 0; i < count; i++)
            {
                instances[i].ScreenPos =
                new Vector3(0, 0, 0);
                instances[i].Scale = 1;
                instances[i].Color = new Vector3(0, 0, 0);
            }
            instanceBuffer = new VertexBuffer(GraphicsDevice, instanceVertexDeclaration, count, BufferUsage.WriteOnly);
            instanceBuffer.SetData(instances);
        }

        public void setEdges(List<Vector3> edges)
        {
            this.edgePositions = edges;
        }
        //Stage 8
        private void UpdateInstanceInformation()
        {

            int stage = 8;
            int lastFrame = 0;
            bool fromOutOfOrder = false;
            SortedList<int, PipelineContainer> outOfOrder = new SortedList<int, PipelineContainer>();
            while (this.dataContainer.Run)
            {
                if (instances == null)
                {
                    continue;
                }
                PipelineContainer container = null;

                while (container == null && this.dataContainer.Run)
                {
                    //container = manager.dequeue(stage);
                    container = null;
                    if (manager.ProcessingQueues[stage].Count > dataContainer.MINFRAMESINCONTAINER)
                    {
                        manager.ProcessingQueues[stage].TryDequeue(out container);
                        if (container == null)
                        {
                            Thread.Sleep(this.dataContainer.SLEEPTIME);
                        }
                    }
                    else
                    {
                        Thread.Sleep(this.dataContainer.SLEEPTIME);
                    }

                }
                if (container == null)
                {
                    break;
                }


                //System.Diagnostics.Debug.WriteLine(container.number + ", " + lastFrame);
                /*if (Math.Abs(container.number - lastFrame) > 20)
                {
                    lastFrame = container.number;
                    
                    foreach (PipelineContainer p in outOfOrder.Values)
                    {
                        manager.recycle.Enqueue(p);
                    }
                    outOfOrder.Clear();
                }

                if (container.number == lastFrame + 1)
                {
                    lastFrame++;
                }
                else if (fromOutOfOrder == false)
                {
                    outOfOrder.Add(container.number, container);
                    if (outOfOrder.Count > 10)
                    {
                        System.Diagnostics.Debug.WriteLine("PCVIEWER has " + outOfOrder.Count + ", " + lastFrame + ", " + container.number);
                        lastFrame++;
                    }
                    continue; //causes lockup
                }*/


                DepthColor[,] depth = container.DC;
                int i = 0;
                for (int x = 0; x < depth.GetLength(0); x++)
                {
                    for (int y = 0; y < depth.GetLength(1); y++)
                    {
                        DepthColor d = depth[x, y];
                        if (d.Depth != 0 && d.Position.Z != 0)
                        {
                            instances[i].ScreenPos = d.Position;
                            instances[i].Scale = 1;
                            if (dataContainer.UseYUV)
                            {
                                d.Color = Vector3.Transform(d.Color, YUVToRGB);
                            }
                            instances[i].Color = d.Color;
                        }
                        else
                        {
                            instances[i].ScreenPos.Z = float.MaxValue;
                            instances[i].Scale = 0f;
                        }
                        i++;
                    }
                }



                this.model = dataContainer.model;
                this.tentativeModels = dataContainer.tentativeModels;


                this.SetBrickTranslate(Matrix.CreateTranslation(container.center));
                Matrix R = container.R;
                switch (dataContainer.trackingConfidence)
                {
                    case TrackingConfidenceLevel.ICPFULL:
                        R = container.R;
                        break;
                    case TrackingConfidenceLevel.ICPTENTATIVE:
                        R = container.R;
                        break;
                    /*case TrackingConfidenceLevel.NORMALS :
                        R = container.normalR;
                        break;*/
                    default:
                        //R = container.R;
                        R = container.R;
                        break;
                }
                //this.SetBrickRotation(container.R);
                this.SetBrickRotation(R);
                Vector3 up = dataContainer.g;
                up.Normalize();
                if (track)
                {
                    CameraPosition = this.brickTranslation.Translation + new Vector3(-400, 75, 340);
                    CameraLookAt = this.brickTranslation.Translation;
                    CameraUp = up;
                }

                this.outlierCenter = dataContainer.outlierCenter;
                foreach (Point v in model.points)
                {
                    Matrix transform = brickRotation * brickTranslation;
                    Vector3 pos = Vector3.Transform(v.position, transform);
                    Matrix onlyRot = new Matrix();
                    onlyRot.M11 = brickRotation.M11;
                    onlyRot.M12 = brickRotation.M12;
                    onlyRot.M13 = brickRotation.M13;
                    onlyRot.M21 = brickRotation.M21;
                    onlyRot.M22 = brickRotation.M22;
                    onlyRot.M23 = brickRotation.M23;
                    onlyRot.M31 = brickRotation.M31;
                    onlyRot.M32 = brickRotation.M32;
                    onlyRot.M33 = brickRotation.M33;
                    float distZ = brickTranslation.M43 + brickRotation.M43;
                    Vector3 transformedNormal = Vector3.Transform(v.normal, onlyRot);
                    //if (i % 5 == 0)
                    if (Vector3.Dot(transformedNormal, Vector3.UnitZ) > this.dataContainer.NORMAL_CULLING_LIMIT)//-0.1f)
                    {
                        instances[i].ScreenPos = pos;
                        instances[i].Scale = 0.4f;
                        instances[i].Color = new Vector3(0, 255, 100);
                    }
                    else
                    {
                        instances[i].ScreenPos = pos;
                        instances[i].Scale = 0.2f;
                        instances[i].Color = new Vector3(255, 0, 100);
                    }
                    i++;
                }
                foreach (Point v in dataContainer.comparisonPoints)
                {
                    instances[i].ScreenPos = v.position;
                    instances[i].Scale = 0.15f;
                    instances[i].Color = new Vector3(0, 255, 100);
                    i++;
                }
                foreach (Point p in container.Qi)
                {
                    Vector3 v = p.position;
                    instances[i].ScreenPos = v;
                    instances[i].Scale = 0.1f;
                    instances[i].Color = new Vector3(255, 0, 0);
                    i++;
                }
                for (int o = i; o < instances.Length; o++)
                {
                    instances[o].ScreenPos.Z = float.MaxValue;
                    instances[o].Scale = 0f;
                }
                Vector3 center = Vector3.Transform(Vector3.One, brickTranslation);

                for (int segment = 0; segment < 90; segment++)
                {
                    instances[i].ScreenPos = center + dataContainer.g * segment;
                    instances[i].Scale = 1;
                    Vector3 c = Vector3.Zero;                    
                    c = new Vector3(0, 0, 255);                    
                    instances[i].Color = c;
                    i++;
                }
                
                /*for (int o = 0; o < 3; o++)
                {
                    if (container.NormalsList[o] != null)
                    {
                        Vector3 normal = container.NormalsList[o];
                        for (int segment = 0; segment < 90; segment++)
                        {
                            instances[i].ScreenPos = center + normal * segment;
                            instances[i].Scale = 1;
                            Vector3 c = Vector3.Zero;
                            if (o == 0)
                            {
                                c = new Vector3(255, 0, 0);
                            }
                            if (o == 1)
                            {
                                c = new Vector3(0, 255, 0);
                            }
                            if (o == 2)
                            {
                                c = new Vector3(0, 0, 255);
                            }
                            instances[i].Color = c;
                            i++;
                        }
                    }
                }*/

                /*if (container.estimatedVectors[0] != null)
                {
                    for (int segment = 0; segment < 90; segment++)
                    {
                        instances[i].ScreenPos = center + container.modelVectors[0] * segment;
                        instances[i].Scale = 1;
                        Vector3 c = Vector3.Zero;

                        instances[i].Color = new Vector3(0.5f, 0, 0);
                        i++;
                    }
                    for (int segment = 0; segment < 90; segment++)
                    {
                        instances[i].ScreenPos = center + container.modelVectors[1] * segment;
                        instances[i].Scale = 1;
                        Vector3 c = Vector3.Zero;

                        instances[i].Color = new Vector3(0, 0.5f, 0);
                        i++;
                    }
                    for (int segment = 0; segment < 90; segment++)
                    {
                        instances[i].ScreenPos = center + container.modelVectors[2] * segment;
                        instances[i].Scale = 1;
                        Vector3 c = Vector3.Zero;

                        instances[i].Color = new Vector3(0, 0, 0.5f);
                        i++;
                    }
                }*/



                instances[i].ScreenPos = center;
                instances[i].Scale = 1;
                instances[i].Color = new Vector3(0, 255, 0);

                container.Timings.Add(DateTime.Now);
                this.manager.Recycle.Enqueue(container);
                this.dataContainer.recordTick();
                this.dataContainer.ICPInliers = container.ICPInliers;
                this.dataContainer.ICPOutliers = container.ICPOutliers;
                this.dataContainer.ICPMSE = container.ICPMSE;
                //System.Diagnostics.Debug.WriteLine(container.number);
                Thread.Sleep(5);
                //this.manager.enqueue(container);
                //manager.processingQueues[++container.stage].Enqueue(container);
                //container = null;
            }


        }

        public static void Main(String[] args)
        {
            Game g = new PCViewer();
            g.Run();
        }



        public PCViewer()
        {
            //transformationUpdater = new Task(() => this.UpdateInstanceInformation());
            this.RGBToYUV = new Matrix(0.299f, 0.587f, 0.144f, 0f, -0.14713f, -0.28886f, 0.436f, 0f, 0.615f, -0.51499f, -0.10001f, 0f, 0f, 0f, 0f, 1f);
            this.RGBToYUV = Matrix.Transpose(this.RGBToYUV);
            this.YUVToRGB = Matrix.Invert(RGBToYUV);
            this.dataContainer = new PipelineDataContainer();
            this.manager = new PipelineManager(this.dataContainer);

            PCViewport = new Viewport();
            PCViewport.X = 0;
            PCViewport.Y = 0;
            PCViewport.Width = 640;
            PCViewport.Height = 480;

            BrickViewport = new Viewport();
            BrickViewport.X = 640;
            BrickViewport.Y = 0;
            BrickViewport.Width = 640;
            BrickViewport.Height = 480;

            graphics = new GraphicsDeviceManager(this);
            graphics.IsFullScreen = false;
            graphics.PreferredBackBufferWidth = 1280;
            graphics.PreferredBackBufferHeight = 480;
            Vector3 cameraDiff, unitX;
            unitX = -Vector3.UnitX;
            cameraDiff = (CameraLookAt - CameraPosition);

            Vector3.Cross(ref cameraDiff, ref unitX, out CameraUp);
            CameraUp.Normalize();
            graphics.ApplyChanges();

            oldScroll = 0;

            Content.RootDirectory = "Content";
            this.IsFixedTimeStep = true;
            graphics.SynchronizeWithVerticalRetrace = true;

            this.model = dataContainer.model;
            Thread Stage6 = new Thread(new ThreadStart(() => UpdateInstanceInformation()));
            Stage6.Start();
        }

        public void setModel(Model m)
        {
            this.model = m;
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
            Projection = Matrix.CreatePerspectiveFieldOfView(43f / 180f * MathHelper.Pi, 640f / 480f, 10, 1500);
            base.Initialize();
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
            brick = Content.Load<XNAModel>("Models\\duploblock");
            secondaryBrick = Content.Load<XNAModel>("Models\\duploblock");

            brickTexture = Content.Load<Texture2D>("Textures\\Exploded_cube_map");


            bindings = new VertexBufferBinding[2];
            bindings[0] = new VertexBufferBinding(geometryBuffer);
            bindings[1] = new VertexBufferBinding(instanceBuffer, 0, 1);

            effect = Content.Load<Effect>("InstancingEffect");


            effect.CurrentTechnique = effect.Techniques["Instancing"];
            effect.Parameters["WVP"].SetValue(View * Projection);
        }

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input, and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {

            // Allows the game to exit


            /*elapsedTime += (int)gameTime.ElapsedGameTime.TotalMilliseconds;
            if (elapsedTime > 1000)
            {
                fps = totalFrames;
                totalFrames = 0;
                elapsedTime = 0;
            }*/
            //.UpdateInstanceInformation();

            HandleInput(Keyboard.GetState(), Mouse.GetState());

            /*if (!freeze)
            {
                if (transformationUpdater.Status != TaskStatus.Running)
                {
                    transformationUpdater.Start();
                }
            }*/
            this.Window.Title = dataContainer.model.tentativeModels.Count + " - " + dataContainer.ModelsWorked + " - minICPRatio:" + dataContainer.currentMaxMSE + " views: " + dataContainer.differentViewCounter + " - " + dataContainer.backgroundEvaluator.ModificationInput.Count + " " + dataContainer.backgroundEvaluator.NormalOutput.Count + "" + dataContainer.backgroundEvaluator.NormalInput.Count + " | " + dataContainer.normalMappings[0] + " " + dataContainer.normalMappings[1] + " " + dataContainer.normalMappings[2] + " " + dataContainer.trackingConfidence + "Ratio: " + Math.Round(dataContainer.ICPMSE, 2) + " Inliers: " + dataContainer.ICPInliers + ", Outliers: " + dataContainer.ICPOutliers + " Total Points: " + (dataContainer.ICPInliers + dataContainer.ICPOutliers) + ", Frametime: " + this.dataContainer.frameTime + "ms" + " Generation Time: " + this.dataContainer.generateTime + "ms";

            base.Update(gameTime);
        }

        protected void UpdateView()
        {
            View = Matrix.CreateLookAt(CameraPosition, CameraLookAt, CameraUp);
        }

        protected void HandleInput(KeyboardState kS, MouseState mS)
        {
            Vector3 CameraLookAtNew;
            int scroll = mS.ScrollWheelValue;
            if (scroll != oldScroll)
            {
                Matrix rotX = new Matrix();
                Vector3 axis = Vector3.Cross(CameraUp, CameraLookAt - CameraPosition);
                axis.Normalize();
                rotX = Matrix.CreateFromAxisAngle(axis, (scroll - oldScroll) / 5000f);

                CameraLookAtNew = CameraLookAt - CameraPosition;
                CameraLookAtNew = Vector3.Transform(CameraLookAtNew, rotX);
                CameraUp = Vector3.Transform(CameraUp, rotX);
                CameraLookAt = CameraPosition + CameraLookAtNew;
                oldScroll = scroll;
            }

            if (kS.IsKeyDown(Keys.A))
            {
                Matrix rotY = new Matrix();
                rotY = Matrix.CreateFromAxisAngle(CameraUp, 0.02f);
                CameraLookAtNew = CameraLookAt - CameraPosition;
                CameraLookAtNew = Vector3.Transform(CameraLookAtNew, rotY);
                //CameraUp = Vector3.Transform(CameraUp, rotY);
                CameraLookAtNew += CameraPosition;
                CameraLookAt = CameraLookAtNew;
            }
            if (kS.IsKeyDown(Keys.D))
            {
                Matrix rotY = new Matrix();
                rotY = Matrix.CreateFromAxisAngle(CameraUp, -0.02f);
                CameraLookAtNew = CameraLookAt - CameraPosition;
                CameraLookAtNew = Vector3.Transform(CameraLookAtNew, rotY);
                //CameraUp = Vector3.Transform(CameraUp, rotY);
                CameraLookAtNew += CameraPosition;
                CameraLookAt = CameraLookAtNew;

            }
            if (kS.IsKeyDown(Keys.W))
            {
                Vector3 direction = CameraLookAt - CameraPosition;
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
            if (kS.IsKeyDown(Keys.Q))
            {
                Vector3 direction = CameraLookAt - CameraPosition;
                direction.Normalize();
                Matrix rot = new Matrix();
                rot = Matrix.CreateFromAxisAngle(direction, -0.02f);
                CameraUp = Vector3.Transform(CameraUp, rot);
            }
            if (kS.IsKeyDown(Keys.E))
            {
                Vector3 direction = CameraLookAt - CameraPosition;
                direction.Normalize();
                Matrix rot = new Matrix();
                rot = Matrix.CreateFromAxisAngle(direction, 0.02f);
                CameraUp = Vector3.Transform(CameraUp, rot);
            }
            if (kS.IsKeyDown(Keys.Space))
            {
                this.freeze = true;
            }
            if (kS.IsKeyDown(Keys.R))
            {
                this.freeze = false;
            }
            if (kS.IsKeyDown(Keys.B))
            {
                this.dataContainer.GenerateBackground = true;
            }
            if (kS.IsKeyDown(Keys.I))
            {
                this.dataContainer.DeNoiseAndICP = true;
            }
            if (kS.IsKeyDown(Keys.V))
            {
                /*CameraUp = new Vector3(0, 1, 1.2f);
                CameraUp.Normalize();
                CameraLookAt = new Vector3(-415, 270, -715);
                CameraPosition = new Vector3(-750, 310, -580);*/
                track = true;
            }

            if (kS.IsKeyDown(Keys.K))
            {
                track = false;
                CameraUp = Vector3.UnitY;
                CameraPosition = new Vector3(0, 0, 0);
                CameraLookAt = new Vector3(0, 0, -370);

            }
            if (kS.IsKeyDown(Keys.R))
            {
                this.manager.processor3D.Reset();
                this.dataContainer.prevNormalR = Matrix.Identity;
            }

            if (kS.IsKeyDown(Keys.L))
            {
                this.manager.processor3D.NormalAligner = true;
            }

            if (kS.IsKeyDown(Keys.M))
            {
                this.dataContainer.EditMode = true;
            }
            if (kS.IsKeyDown(Keys.N))
            {
                this.dataContainer.EditMode = false;
            }
            if (kS.IsKeyDown(Keys.Enter))
            {
                this.dataContainer.Attach = true;
                this.dataContainer.currentMaxMSE = dataContainer.ICPMSE+2;
            }
            if (kS.IsKeyDown(Keys.O))
            {
                this.dataContainer.WrongModel = true;
            }

            if (kS.IsKeyDown(Keys.NumPad5))
            {
                this.dataContainer.ApplyModel = true;
            }

            if (kS.IsKeyDown(Keys.NumPad1))
            {
                this.dataContainer.addColor = BrickColor.RED;
            }

            if (kS.IsKeyDown(Keys.NumPad2))
            {
                this.dataContainer.addColor = BrickColor.GREEN;
            }

            if (kS.IsKeyDown(Keys.NumPad3))
            {
                this.dataContainer.addColor = BrickColor.BLUE;
            }

            if (kS.IsKeyDown(Keys.NumPad4))
            {
                this.dataContainer.addColor = BrickColor.YELLOW;
            }

            if (kS.IsKeyDown(Keys.NumPad0))
            {
                this.dataContainer.RevertToOld = true;
            } 


            UpdateView();
        }

        private void GenerateGeometryBuffers()
        {
            VertexPositionTexture[] vertices = new VertexPositionTexture[24];
            vertices[0].Position = new Vector3(-1, 1, -1);
            vertices[0].TextureCoordinate = new Vector2(0, 0);
            vertices[1].Position = new Vector3(1, 1, -1);
            vertices[1].TextureCoordinate = new Vector2(1, 0);
            vertices[2].Position = new Vector3(-1, 1, 1f);
            vertices[2].TextureCoordinate = new Vector2(0, 1);
            vertices[3].Position = new Vector3(1, 1, 1f);
            vertices[3].TextureCoordinate = new Vector2(1, 1);

            vertices[4].Position = new Vector3(-1, -1, 1f);
            vertices[4].TextureCoordinate = new Vector2(0, 0);
            vertices[5].Position = new Vector3(1, -1, 1f);
            vertices[5].TextureCoordinate = new Vector2(1, 0);
            vertices[6].Position = new Vector3(-1, -1, -1);
            vertices[6].TextureCoordinate = new Vector2(0, 1);
            vertices[7].Position = new Vector3(1, -1, -1);
            vertices[7].TextureCoordinate = new Vector2(1, 1);

            vertices[8].Position = new Vector3(-1, 1, -1);
            vertices[8].TextureCoordinate = new Vector2(0, 0);
            vertices[9].Position = new Vector3(-1, 1, 1f);
            vertices[9].TextureCoordinate = new Vector2(1, 0);
            vertices[10].Position = new Vector3(-1, -1, -1);
            vertices[10].TextureCoordinate = new Vector2(0, 1);
            vertices[11].Position = new Vector3(-1, -1, 1f);
            vertices[11].TextureCoordinate = new Vector2(1, 1);

            vertices[12].Position = new Vector3(-1, 1, 1f);
            vertices[12].TextureCoordinate = new Vector2(0, 0);
            vertices[13].Position = new Vector3(1, 1, 1f);
            vertices[13].TextureCoordinate = new Vector2(1, 0);
            vertices[14].Position = new Vector3(-1, -1, 1f);
            vertices[14].TextureCoordinate = new Vector2(0, 1);
            vertices[15].Position = new Vector3(1, -1, 1f);
            vertices[15].TextureCoordinate = new Vector2(1, 1);

            vertices[16].Position = new Vector3(1, 1, 1f);
            vertices[16].TextureCoordinate = new Vector2(0, 0);
            vertices[17].Position = new Vector3(1, 1, -1);
            vertices[17].TextureCoordinate = new Vector2(1, 0);
            vertices[18].Position = new Vector3(1, -1, 1f);
            vertices[18].TextureCoordinate = new Vector2(0, 1);
            vertices[19].Position = new Vector3(1, -1, -1);
            vertices[19].TextureCoordinate = new Vector2(1, 1);

            vertices[20].Position = new Vector3(1, 1, -1);
            vertices[20].TextureCoordinate = new Vector2(0, 0);
            vertices[21].Position = new Vector3(-1, 1, -1);
            vertices[21].TextureCoordinate = new Vector2(1, 0);
            vertices[22].Position = new Vector3(1, -1, -1);
            vertices[22].TextureCoordinate = new Vector2(0, 1);
            vertices[23].Position = new Vector3(-1, -1, -1);
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

        }
        protected override void OnExiting(Object sender, EventArgs args)
        {
            base.OnExiting(sender, args);
            this.dataContainer.Run = false;
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Viewport = PCViewport;
            if (dataContainer.EditMode)
            {
                GraphicsDevice.Clear(Microsoft.Xna.Framework.Color.Orange);
            }
            else if (dataContainer.ApplyModel)
            {
                GraphicsDevice.Clear(Microsoft.Xna.Framework.Color.Purple);
            }
            else
            {
                GraphicsDevice.Clear(Microsoft.Xna.Framework.Color.CornflowerBlue);
            }


            effect.CurrentTechnique = effect.Techniques["Instancing"];
            effect.Parameters["WVP"].SetValue(View * Projection);

            GraphicsDevice.SetVertexBuffers(null);
            instanceBuffer.SetData(instances);

            GraphicsDevice.Indices = indexBuffer;

            effect.CurrentTechnique.Passes[0].Apply();

            GraphicsDevice.SetVertexBuffers(bindings);

            GraphicsDevice.DrawInstancedPrimitives(PrimitiveType.TriangleList, 0, 0, 24, 0, 12, count);
            base.Draw(gameTime);

            GraphicsDevice.Viewport = BrickViewport;

            Matrix[] transforms = new Matrix[brick.Bones.Count];
            brick.CopyAbsoluteBoneTransformsTo(transforms);

            foreach (LocatedBrick b in model.Bricks)
            {
                foreach (ModelMesh mesh in brick.Meshes)
                {

                    foreach (BasicEffect eff in mesh.Effects)
                    {
                        eff.TextureEnabled = false;
                        eff.DiffuseColor = b.getColor() * 0.7f;
                        eff.Alpha = 1;
                        eff.EnableDefaultLighting();
                        eff.World = transforms[mesh.ParentBone.Index] * Matrix.CreateScale(10f) * b.getTransformation() * Matrix.CreateTranslation(this.offset) * this.brickRotation * brickTranslation;// *

                        eff.View = View;
                        eff.Projection = Projection;
                    }

                    mesh.Draw();
                }
            }

            
            if (tentativeModels != null && dataContainer.EditMode)
            {
                SortedDictionary<float, TentativeModel> cachedTentativeModels = new SortedDictionary<float, TentativeModel>(tentativeModels);
                for (int i = 0; i < 10; i++)
                {
                    if (cachedTentativeModels.Keys.Count > i)
                    {
                        float key = cachedTentativeModels.Keys.ElementAt(i);
                        TentativeModel tentativeModel = cachedTentativeModels[key];
                        if (tentativeModel.TentativeBrick != null)
                        {
                            foreach (ModelMesh mesh in secondaryBrick.Meshes)
                            {

                                foreach (BasicEffect eff in mesh.Effects)
                                {
                                    eff.TextureEnabled = false;
                                    eff.DiffuseColor = tentativeModel.TentativeBrick.getColor();
                                    eff.Alpha = 0.6f;
                                    eff.EnableDefaultLighting();
                                    eff.World = transforms[mesh.ParentBone.Index] * Matrix.CreateScale(10f) * tentativeModel.TentativeBrick.getTransformation() * Matrix.CreateTranslation(this.offset) * this.brickRotation * brickTranslation;// *

                                    eff.View = View;
                                    eff.Projection = Projection;
                                }
                                mesh.Draw();
                            }
                        }
                    }
                }
            }

            if (dataContainer.EditMode)
            {
                foreach (ModelMesh mesh in brick.Meshes)
                {

                    foreach (BasicEffect eff in mesh.Effects)
                    {
                        eff.TextureEnabled = false;
                        eff.DiffuseColor = Vector3.One * 0.7f;
                        eff.EnableDefaultLighting();
                        eff.World = transforms[mesh.ParentBone.Index] * Matrix.CreateScale(10f) * Matrix.CreateTranslation(this.offset) * Matrix.CreateTranslation(this.outlierCenter);// *

                        eff.View = View;
                        eff.Projection = Projection;
                    }

                    mesh.Draw();
                }
            }
            base.Draw(gameTime);

        }

    }
}
