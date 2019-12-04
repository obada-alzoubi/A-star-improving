using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using System.Drawing;

namespace PathFinder
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        public static int width = 0;
        public static int height = 0;
        public static string[] map;
        public static string[] scen;
        public static string[,] scenInfo;
        public static string mapName;
        [STAThread]
        static void Main()
        {

            string[] mapPath = Directory.GetFiles(@"c:\benchmark\map", "*.map");
            string[] scenPath = Directory.GetFiles(@"c:\benchmark\map", "*.scen");
            DirectoryInfo dinfo = new DirectoryInfo(@"c:\benchmark\map");
            FileInfo[] Files = dinfo.GetFiles("*.map");
            // Loop over all maps in the directory 
            for (int k = 0; k < Files.Length; k++)
            {
                mapName = Files[k].Name;

                // of the array is one line of the file. 
                map = System.IO.File.ReadAllLines(mapPath[k]);
                // Display the file contents by using a foreach loop.
                // Keep the console window open in debug mode.
                ScenarioLoader loader = new ScenarioLoader(scenPath[k]);
                scen = ScenarioLoader.scen;
                scenInfo = ScenarioLoader.scenInfo;
                //Reading information 
                string[] readHiegth = map[1].Split(' ');
                string[] readWidth = map[2].Split(' ');
                Console.WriteLine("Map Name: {0}", mapName);
                Console.WriteLine("The Heigth of the Map: {0}", readHiegth[1]);
                Console.WriteLine("The Width of the Map: {0}", readWidth[1]);
                //Build the map with Width and Height shown in Map File 
                Map mapBuild = new Map(map, int.Parse(readWidth[1]), int.Parse(readHiegth[1]));
                State[,] builtMap = mapBuild.MapBuild();
                //Retrieve information from scenario file 

                //Search The Map 
                //CoreClass Searcher = new CoreClass(builtMap, int.Parse(readWidth[1]), int.Parse(readHiegth[1]), 210, 55, 185, 241);


                //Searcher.Search2(builtMap, startPoint, endPoint);
                ResetCoreClass();
                for (int i = 0; i < scenInfo.GetLongLength(0); i++)
                {

                    Console.WriteLine("Scenario No. {0}:", i);
                    if (int.Parse(scenInfo[i, 4]) > int.Parse(readWidth[1]) - 3 || int.Parse(scenInfo[i, 5]) > int.Parse(readHiegth[1]) - 3 || int.Parse(scenInfo[i, 7]) > int.Parse(readWidth[1]) - 3 || int.Parse(scenInfo[i, 6]) > int.Parse(readHiegth[1]) - 3)
                        continue;
                    if (int.Parse(scenInfo[i, 4]) < 3 || int.Parse(scenInfo[i, 5]) < 3 || int.Parse(scenInfo[i, 7]) < 3 || int.Parse(scenInfo[i, 6]) < 3)
                        continue;
                    CoreClass Searcher2 = new CoreClass(builtMap, int.Parse(readWidth[1]), int.Parse(readHiegth[1]), int.Parse(scenInfo[i, 5]), int.Parse(scenInfo[i, 4]), int.Parse(scenInfo[i, 7]), int.Parse(scenInfo[i, 6]));
                    Point startPoint = new Point(int.Parse(scenInfo[i, 5]), int.Parse(scenInfo[i, 4]));
                    Point endPoint = new Point(int.Parse(scenInfo[i, 7]), int.Parse(scenInfo[i, 6]));

                    //CoreClass Searcher2 = new CoreClass(builtMap, int.Parse(readWidth[1]), int.Parse(readHiegth[1]), 695, 101, 695 ,212);
                    try
                    { 
                        Searcher2.Search2(builtMap, startPoint, endPoint);
                    }
                    catch
                    {
                        Console.WriteLine("Scenario No. : {0} was not completed", i);
                    }
                    CoreClass.coutTime = 0;
                }
                Console.WriteLine("Press any key to exit.");
                //System.Console.ReadKey();
                //For Methods 
                //Application.EnableVisualStyles();
                //Application.SetCompatibleTextRenderingDefault(false);
                //Application.Run(new PathFinding());
            }
        }
        public static void ResetCoreClass()
        {
            CoreClass.ExpandedNodes = 0;
            //CoreClass.FirstCutCost.Initialize();
            //CoreClass.Mindiff = 0;
            CoreClass.NumberCuts = 0;
            CoreClass.NumBorNode = 0;
            CoreClass.PathHitBorder = false;
            CoreClass.pathStates.Clear();
            //CoreClass.ProcessedFirstCut.Clear();
            CoreClass.PureAStar = false;
            CoreClass.time = TimeSpan.Zero;
            CoreClass.pureClosedList.Clear();
            CoreClass.ProcessedList.Clear();
            CoreClass.PureExpandedNodes = 0;
            CoreClass.coutTime = 0;
            
            //CoreClass.AStarPathStates.Clear();
        }

    }
}
