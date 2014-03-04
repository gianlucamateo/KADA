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
using System.Threading.Tasks;

namespace KADA
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class PCViewer : Microsoft.Xna.Framework.Game
    {
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;

        IndexBuffer Indices;

        private Matrix World;
        private Matrix View;
        private Matrix Projection;
        VertexBuffer Square;
        VertexBuffer Transformations;
        Effect effect;
        Matrix[] Trans;
        int counter;
        Task transformationUpdater;

        VertexDeclaration Transformation = new VertexDeclaration
        (
            new VertexElement(0, VertexElementFormat.Vector4, VertexElementUsage.TextureCoordinate, 0),
            new VertexElement(16, VertexElementFormat.Vector4, VertexElementUsage.TextureCoordinate, 1),
            new VertexElement(32, VertexElementFormat.Vector4, VertexElementUsage.TextureCoordinate, 2),
            new VertexElement(48, VertexElementFormat.Vector4, VertexElementUsage.TextureCoordinate, 3)
        );

        public PCViewer()
        {
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
            View = Matrix.CreateLookAt(new Vector3(0, 0, 1000), new Vector3(0, 0, 0), Vector3.Up);
            Projection = Matrix.CreatePerspectiveFieldOfView(MathHelper.PiOver4, GraphicsDevice.Viewport.AspectRatio, 1, 100000);
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
            this.SetUpVertexBuffer();
            this.SetUpIndexBuffer();
            effect = Content.Load<Effect>("InstancingEffect");

            effect.Parameters["World"].SetValue(World);
            effect.Parameters["View"].SetValue(View);
            effect.Parameters["Projection"].SetValue(Projection);

            this.SetUpTransformations();



            // TODO: use this.Content to load your game content here
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
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input, and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {
            // Allows the game to exit
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed)
                this.Exit();

            // TODO: Add your update logic here
            transformationUpdater = new Task(() => this.UpdateTransformations());
            transformationUpdater.Start();



            base.Update(gameTime);
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            lock (this.GraphicsDevice)
            {
                GraphicsDevice.Clear(Color.CornflowerBlue);
                GraphicsDevice.SetVertexBuffers(new VertexBufferBinding(Square, 0, 0), new VertexBufferBinding(Transformations, 0, 1));
                GraphicsDevice.Indices = Indices;

                foreach (EffectPass pass in effect.CurrentTechnique.Passes)
                {
                    pass.Apply();

                    if (Transformations != null)
                    {
                        GraphicsDevice.DrawInstancedPrimitives(PrimitiveType.TriangleStrip, 0, 0, 4, 0, 2, Transformations.VertexCount);
                    }
                }

                base.Draw(gameTime);
            }
        }

        private void SetUpVertexBuffer()
        {
            VertexPositionColor[] Vertices = new VertexPositionColor[4];
            Vertices[0].Position = new Vector3(0, 0, 0);
            Vertices[0].Color = new Color(0, 255, 0, 1);
            Vertices[1].Position = new Vector3(1, 0, 0);
            Vertices[1].Color = new Color(0, 1, 255, 1);
            Vertices[2].Position = new Vector3(0, 1, 0);
            Vertices[2].Color = new Color(255, 1, 0, 1);
            Vertices[3].Position = new Vector3(1, 1, 0);
            Vertices[3].Color = new Color(255, 255, 0, 1);
            Square = new VertexBuffer(GraphicsDevice, VertexPositionColor.VertexDeclaration, 4, BufferUsage.WriteOnly);
            Square.SetData(Vertices);
        }

        private void SetUpIndexBuffer()
        {
            Indices = new IndexBuffer(GraphicsDevice, typeof(uint), 4, BufferUsage.WriteOnly);
            Indices.SetData(new uint[] { 0, 2, 1, 3 });
        }

        private void UpdateTransformations()
        {
            counter++;
            int xMax = 640;
            int yMax = 480;
            int i = 0;
            for (int x = 0; x < xMax; x++)
            {
                for (int y = 0; y < yMax; y++)
                {
                    Trans[i] = Matrix.CreateTranslation(x - (xMax / 2), y - (yMax / 2), (float)counter * (x+y) / 1000f);
                    i++;
                }
            }
            lock (this.GraphicsDevice)
            {
                GraphicsDevice.SetVertexBuffer(null);
                Transformations.SetData(Trans);
            }

        }

        private void SetUpTransformations()
        {
            int xMax = 640;
            int yMax = 480;
            Trans = new Matrix[640 * 480];
            int i = 0;
            for (int x = 0; x < xMax; x++)
            {
                for (int y = 0; y < yMax; y++)
                {
                    Trans[i] = Matrix.CreateTranslation(x - (xMax / 2), y - (yMax / 2), 0);
                    i++;
                }

            }

            Transformations = new VertexBuffer(GraphicsDevice, Transformation, Trans.Length, BufferUsage.WriteOnly);
            Transformations.SetData(Trans);
        }


    }
}
