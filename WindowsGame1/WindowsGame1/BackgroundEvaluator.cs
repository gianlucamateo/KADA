using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using Microsoft.Xna.Framework;
using System.Collections.Concurrent;
using System.IO;
using Point = KADA.Point;
using System.Threading.Tasks;


namespace KADA
{
    public class BackgroundEvaluator
    {
        private PipelineDataContainer dataContainer;
        public ConcurrentQueue<PipelineContainer> NormalInput;
        public ConcurrentQueue<List<Point>> ModificationInput;
        public ConcurrentQueue<Matrix> NormalOutput;
        public ConcurrentQueue<List<Point>> PointsInput;
        private List<BrickColor> possibleColors;
        private Matrix[] possibleRotations;
        private Model oldModel;
        public bool ModificationRunning = false;
        public SortedDictionary<float, Model> guesses;



        public BackgroundEvaluator(PipelineDataContainer dataContainer)
        {
            this.possibleColors = new List<BrickColor>();
            this.PointsInput = new ConcurrentQueue<List<Point>>();
            this.NormalOutput = new ConcurrentQueue<Matrix>();
            this.NormalInput = new ConcurrentQueue<PipelineContainer>();
            this.ModificationInput = new ConcurrentQueue<List<Point>>();
            this.guesses = new SortedDictionary<float, Model>();
            this.dataContainer = dataContainer;
            possibleRotations = new Matrix[64];

            Matrix XRot = Matrix.CreateRotationX((float)Math.PI / 2);
            Matrix YRot = Matrix.CreateRotationY((float)Math.PI / 2);
            Matrix ZRot = Matrix.CreateRotationZ((float)Math.PI / 2);

            possibleColors.Add(BrickColor.RED);
            possibleColors.Add(BrickColor.GREEN);
            possibleColors.Add(BrickColor.BLUE);
            possibleColors.Add(BrickColor.YELLOW);

            int x = 0, y = 0, z = 0;
            int count = 0;
            for (int i = 0; i < 64; i++)
            {
                Matrix rot = Matrix.Identity;
                for (int xCount = 0; xCount < x; xCount++)
                {
                    rot = XRot * rot;
                }

                for (int yCount = 0; yCount < y; yCount++)
                {
                    rot = YRot * rot;
                }

                for (int zCount = 0; zCount < z; zCount++)
                {
                    rot = ZRot * rot;
                }
                possibleRotations[count++] = rot;
                x++;
                if (x == 4)
                {
                    x = 0;
                    y++;
                    if (y == 4)
                    {
                        y = 0;
                        z++;
                        if (z == 4)
                        {
                            break;
                        }
                    }
                }
            }
            Thread normalAnalyzer = new Thread(new ThreadStart(() => analyzeNormal()));
            //normalAnalyzer.Start();

            Thread blockTracker = new Thread(new ThreadStart(() => trackAdditionalBlock()));
            blockTracker.Start();

            Thread modificationApplier = new Thread(new ThreadStart(() => applyModification()));
            modificationApplier.Start();

        }

        private void applyModification()
        {
            List<Point> container;
            while (this.dataContainer.Run)
            {
                if (this.dataContainer.RevertToOld)
                {
                    dataContainer.model = oldModel;
                    dataContainer.RevertToOld = false;
                }
                if (this.dataContainer.WrongModel)
                {
                    dataContainer.EditMode = true;
                    float tempKey = this.guesses.Keys.ElementAt(this.guesses.Keys.Count - 1);
                    Model temp = this.guesses[tempKey];

                    Model nextTry = new Model(true, temp.center, temp.Bricks, true);
                    dataContainer.model = nextTry;
                    dataContainer.comparisonPoints = dataContainer.model.points;
                    this.guesses.Remove(tempKey);
                    Thread.Sleep(300);
                    this.dataContainer.WrongModel = false;
                    dataContainer.EditMode = false;

                }
                if (this.dataContainer.ApplyModel)
                {
                    Model temp = new Model(true, Vector3.Zero, dataContainer.model.Bricks, false);
                    dataContainer.model = temp;
                    dataContainer.ApplyModel = false;
                }
                ModificationInput.TryDequeue(out container);
                if (container == null)
                {
                    Thread.Sleep(100);
                    continue;
                }
                if (dataContainer.EditMode == false)
                {
                    continue;
                }
                this.ModificationRunning = true;
                this.dataContainer.Attach = false;

                while (this.ModificationInput.Count > 0)
                {
                    List<Point> dumpContainer;
                    this.ModificationInput.TryDequeue(out dumpContainer);
                }
                Console.WriteLine("working");
                oldModel = dataContainer.model;
                this.guesses.Clear();
                List<Point> qi = new List<Point>(container);
                SortedDictionary<float, TentativeModel> dict = new SortedDictionary<float, TentativeModel>(dataContainer.tentativeModels);

                float maxRatio = float.MinValue;
                int i = 0;
                Model currentModel = null;
                ConcurrentDictionary<float, Model> concurrentDict = new ConcurrentDictionary<float, Model>();
                Parallel.ForEach(dict.Keys, key =>
                //foreach (float key in dict.Keys)
                {
                    bool skip = false;
                    i++;
                    dataContainer.ModelsWorked = i;
                    if (i++ > 100)
                    {
                        skip = true;
                    }
                    /*if (!this.dataContainer.Run)
                    {
                        return;
                    }*/


                    if (!skip)
                    {
                        Console.WriteLine("working on model");
                        Model model = dict[key].validate(dataContainer.addColor);
                        KDTreeWrapper tree = model.getKDTree();

                        int count = 0;
                        float MSE = 0;
                        //Matrix Rinv = Matrix.Invert(dataContainer.R);
                        int dismiss = 0;
                        int inliers = 0;
                        foreach (Point p in qi)
                        {
                            /*if (dismiss++ % 5 == 0)
                            {
                                continue;
                            }*/
                            Vector3 transformedP = Vector3.Zero;

                            transformedP = p.position;//Vector3.Transform(p.position - dataContainer.center, Rinv);
                            double[] arr = { transformedP.X, transformedP.Y, transformedP.Z };
                            KDTree.NearestNeighbor<Point> neighbor = tree.NearestNeighbors(arr, 1, -1);
                            neighbor.MoveNext();
                            Point point = neighbor.Current;
                            bool foundMatch = false;
                            foreach (BrickColor bc in possibleColors)
                            {
                                int number = (int)bc;
                                if ((p.brickColorInteger / number) * number == p.brickColorInteger)
                                {
                                    if (point.brickColor == bc)
                                    {
                                        foundMatch = true;
                                    }
                                }
                            }
                            if (neighbor.CurrentDistance < 100)
                            {
                                inliers++;
                            }
                            if (!foundMatch)
                            {
                                MSE += 40000;
                            }
                            count++;
                            
                            {
                                MSE += (float)neighbor.CurrentDistance;
                            }
                           
                        }
                        MSE /= ((float)count + 1f);
                        
                        model.kdTree = null;
                        model.points = null;
                        model.tentativeModels = null;
                        model.voxelGrid = null;
                        model.pointGrid = null;
                        /*if (ratio > maxRatio)
                        {
                            maxRatio = ratio;
                            currentModel = model;
                        }*/
                        
                        while (!concurrentDict.TryAdd(1 / MSE, model) && MSE < 1000000)//concurrentDict.ContainsKey(ratio) && ratio < 1000000)
                        {
                            MSE += 0.01f;
                            //Console.WriteLine(ratio);
                        }
                        /*if (ratio <= 1000000)
                        {
                            concurrentDict.TryAdd(ratio,model);//Add(ratio, model);
                        }*/
                        while (concurrentDict.Keys.Count > 30)
                        {
                            List<float> list = concurrentDict.Keys.ToList();
                            list.Sort();
                            Model modelDump;
                            int tries = 0;
                            while (!concurrentDict.TryRemove(list[0], out modelDump) && tries < 10)
                            {
                                tries++;
                            }
                            //this.guesses.Remove(k);
                        }
                    }

                });
                //this.guesses.Remove(this.guesses.Keys.ElementAt(this.guesses.Keys.Count-1));
                List<float> fromConcurrent = concurrentDict.Keys.ToList();
                foreach (float key in fromConcurrent)
                {
                    Model m;
                    while (!concurrentDict.TryGetValue(key, out m)) ;
                    this.guesses.Add(key, m);
                }
                currentModel = this.guesses[this.guesses.Keys.ElementAt(this.guesses.Keys.Count - 1)];
                Model finalModel = new Model(true, currentModel.center, currentModel.Bricks, true);
                dataContainer.model = finalModel;
                dataContainer.comparisonPoints = finalModel.points;
                dataContainer.EditMode = false;
                this.ModificationRunning = false;

            }
        }

        private void analyzeNormal()
        {
            StreamWriter file = new StreamWriter("rotationScores.txt");
            Thread.Sleep(10000);
            List<Vector3> points = new List<Vector3>();
            PipelineContainer container = null;
            KADA.KDTreeWrapper kdTree = dataContainer.model.getKDTree();
            double[] searchArr = new double[3];
            List<double> values = new List<double>();
            Matrix bestRot = Matrix.Identity;
            Vector3 v;
            while (this.dataContainer.Run)
            {
                NormalInput.TryDequeue(out container);
                if (container == null)
                {
                    Thread.Sleep(dataContainer.SLEEPTIME);
                    continue;
                }

                List<Point> qi = new List<Point>(container.Qi);
                points.Clear();
                Matrix R = container.rawNormalR;
                R = Matrix.Invert(R);
                foreach (Point point in qi)
                {
                    v = point.position;
                    Vector3 p = v - container.center;
                    points.Add(Vector3.Transform(p, R));
                }
                double minDist = double.MaxValue;


                Random rand = new Random();
                double totalDist = 0;
                int pointsCalculated = 0;
                int count = 0;
                double variance = 0;
                double avg = 0;
                double measure = 0;
                foreach (Matrix pRot in this.possibleRotations)
                {

                    Matrix rot = Matrix.Invert(pRot);
                    totalDist = 0;
                    pointsCalculated = 0;
                    variance = 0;
                    values.Clear();
                    foreach (Vector3 p in points)
                    {
                        Vector3 point = p;

                        point = Vector3.Transform(point, rot);
                        if (rand.NextDouble() < 0.2)
                        {
                            searchArr[0] = point.X;
                            searchArr[1] = point.Y;
                            searchArr[2] = point.Z;
                            KDTree.NearestNeighbor<Point> neighbour = kdTree.NearestNeighbors(searchArr, 1);
                            neighbour.MoveNext();
                            totalDist += neighbour.CurrentDistance;
                            pointsCalculated++;
                            values.Add(neighbour.CurrentDistance);
                        }
                    }
                    avg = totalDist / pointsCalculated;

                    foreach (double value in values)
                    {
                        variance += (value - avg) * (value - avg);
                    }

                    variance /= pointsCalculated;
                    measure = avg + 2 * variance;
                    file.WriteLine(++count + " : " + avg + " and var: " + variance + " and measure: " + measure);
                    if (measure < minDist)
                    {
                        minDist = measure;
                        bestRot = pRot;
                    }
                }
                NormalOutput.Enqueue(bestRot);


            }
        }

        private void trackAdditionalBlock()
        {
            Vector3 oldCenter = Vector3.Zero;
            List<Point> PointsList;
            //List<Vector3> Outliers = new List<Vector3>();
            int[,] grid = new int[200, 200], outputGrid = new int[200, 200];
            bool[,] excludeFromErosion = new bool[200, 200];
            float[,] zVals = new float[200, 200];
            SortedDictionary<float, TentativeModel> dict = new SortedDictionary<float, TentativeModel>();
            while (this.dataContainer.Run)
            {
                PointsInput.TryDequeue(out PointsList);
                if (PointsList == null)
                {
                    Thread.Sleep(3 * dataContainer.SLEEPTIME);
                    continue;
                }
                //Outliers.Clear();
                for (int X = 0; X < grid.GetLength(0); X++)
                {
                    for (int Y = 0; Y < grid.GetLength(1); Y++)
                    {
                        grid[X, Y] = 0;
                        zVals[X, Y] = 0;
                        outputGrid[X, Y] = 0;
                        excludeFromErosion[X, Y] = false;
                    }
                }
                //float minX = float.MaxValue, maxX = 0, minY = float.MaxValue, maxY = 0;

                foreach (TentativeModel t in dataContainer.model.tentativeModels)
                {
                    t.TentativeBrick.setColor(dataContainer.addColor);
                }
                int[] colors = new int[8];
                foreach (Point p in PointsList)
                {

                    int X = (int)Math.Floor(p.position.X / 10) + grid.GetLength(0) / 2;
                    int Y = (int)Math.Floor(p.position.Y / 10) + grid.GetLength(1) / 2;
                    if (p.brickColorInteger < 8)
                        colors[p.brickColorInteger]++;

                    grid[X, Y]++;
                    zVals[X, Y] += p.position.Z;
                    if (p.ConsideredICP)
                    {
                        excludeFromErosion[X, Y] = true;
                    }

                }
                for (int X = 0; X < grid.GetLength(0); X++)
                {
                    for (int Y = 0; Y < grid.GetLength(1); Y++)
                    {
                        if (grid[X, Y] > 0)
                        {
                            zVals[X, Y] /= grid[X, Y];
                            grid[X, Y] = 1;
                        }

                    }
                }

                int finalColor = 0;
                int maxVal = 0;
                for (int i = 0; i < 8; i++)
                {
                    if (colors[i] > maxVal)
                    {
                        finalColor = i;
                        maxVal = colors[i];
                    }
                }
                if (maxVal > 50)
                    dataContainer.addColor = (BrickColor)finalColor;
                _2DProcessor.Erode(grid.GetLength(0), grid.GetLength(1), grid, outputGrid);
                for (int X = 0; X < grid.GetLength(0); X++)
                {
                    for (int Y = 0; Y < grid.GetLength(1); Y++)
                    {
                        if (excludeFromErosion[X, Y])
                        {
                            outputGrid[X, Y] = grid[X, Y];
                        }
                    }
                }


                Vector3 pos = Vector3.Zero;
                int count = 0;
                for (int X = 0; X < outputGrid.GetLength(0); X++)
                {
                    for (int Y = 0; Y < outputGrid.GetLength(1); Y++)
                    {
                        if (outputGrid[X, Y] > 0)
                        {
                            count++;
                            Vector3 currentPos = new Vector3((X - outputGrid.GetLength(0) / 2) * 10, (Y - outputGrid.GetLength(0) / 2) * 10, zVals[X, Y]);
                            pos += currentPos;
                        }
                    }
                }
                pos /= count;
                if (pos.Length() > 0)
                {
                    oldCenter = pos;
                }
                else
                {
                    pos = oldCenter;
                }
                pos += (this.dataContainer.g * 18);
                dataContainer.outlierCenter = pos;
                dict.Clear();
                TentativeModel bestGuess = null;
                float distance = float.MaxValue;
                foreach (TentativeModel model in dataContainer.model.tentativeModels)
                {
                    Vector3 p = model.TentativeBrick.center;
                    p = Vector3.Transform(p, dataContainer.R);
                    p += dataContainer.center;
                    float tentativeDistance = Math.Abs((p - pos).Length());
                    /*if (tentativeDistance < distance)
                    {
                        distance = tentativeDistance;
                        bestGuess = model;
                    }*/
                    if (!float.IsNaN(tentativeDistance) && tentativeDistance < 1000)
                    {
                        while (dict.ContainsKey(tentativeDistance))
                        {
                            tentativeDistance += 0.0001f;
                        }
                        dict.Add(tentativeDistance, model);
                    }
                }
                //dataContainer.tentativeModel = bestGuess;
                dataContainer.tentativeModels = new SortedDictionary<float, TentativeModel>(dict);
                //Console.WriteLine(distance);

            }


        }


    }

}
