using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;
namespace PathFinder
{
    public class Cutter
    {
        //Define the Region of High Cost Region 
        Point Right;
        Point Left;
        Point startState;
        Point endState;
        int height;
        int width;
        //Variable to Deal with graph
        public State[,] map;
        //To View Border Nodes
        public static List<Point> BorderList = new List<Point>();
        //To Show if we Reached the Maximum Cut Size
        public static bool FullPathExplored = false;
        //Store Boder Sides to Compute Ratios
        public static List<Point> LeftSide = new List<Point>();
        public static List<Point> RightSide = new List<Point>();
        public static List<Point> UpSide = new List<Point>();
        public static List<Point> DownSide = new List<Point>();
        //Default Constructor
        public Cutter(State[,] map,int height,int width, Point startState ,Point endState)
        {
           
            this.startState=startState;
            this.endState=endState;
            this.height = height;
            this.width = width;
            //map = new State[height, width];
            this.map = map;
        }

        //General Repair Method
        public State[,] Repair(Point Left,Point Right)
        {

            this.Left = Left;
            this.Right = Right;
            //Clear Border Side List for Each New Cut
            LeftSide.Clear();
            RightSide.Clear();
            UpSide.Clear();
            DownSide.Clear();
            this.Left = Left;
            this.Right = Right;
            //graph Variable
            State[,] mapR = new State[height, width];
            //Copy the graph
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    //Copy objects
                    mapR[i, j] = new State(0, 0, 0, 0, 0, 0, false, "X",-1,-1);
                    mapR[i, j].cost = map[i, j].cost;
                    mapR[i, j].heuristic = map[i, j].heuristic;
                    mapR[i, j].isBorder = map[i, j].isBorder;
                    mapR[i, j].processedCount = map[i, j].processedCount;
                    mapR[i, j].totalCost = map[i, j].totalCost;
                    mapR[i, j].value = map[i, j].value;
                    mapR[i, j].X = map[i, j].X;
                    mapR[i, j].Y = map[i, j].Y;
                    mapR[i, j].previous_X = map[i, j].previous_X;
                    mapR[i, j].previous_Y = map[i, j].previous_Y;

                }

            }
            if (FullPathExplored)
            {
                //Return the full graph if We reached The Maximum Size
                return mapR;
            }
            else
            {
                //Make out of Cut untraversable
                for (int i = 0; i < Left.X - 1; i++)
                {
                    for (int j = 0; j < width; j++)
                    {
                        //make it unwalkable,we used out of boundries @
                        mapR[i, j].value = "@";
                    }
                }

                for (int i = (int)Right.X + 1; i < height; i++)
                {
                    for (int j = 0; j < width; j++)
                    {
                        //make it unwalkable,we used out of boundries @
                        mapR[i, j].value = "@";
                    }
                }
                for (int i = (int)Left.X - 1; i <= Right.X + 1; i++)
                {
                    for (int j = 0; j < Left.Y - 1; j++)
                    {

                        //make it unwalkable,we used out of boundries @
                        mapR[i, j].value = "@";
                    }
                }

                for (int i = (int)Left.X - 1; i <= Right.X + 1; i++)
                {
                    for (int j = (int)Right.Y + 1; j < width; j++)
                    {
                        //make it unwalkable,we used out of boundries @
                        mapR[i, j].value = "@";
                    }
                }


                //Build Border Nodes 
                //Build Down Side 
                for (int i = (int)Left.X - 1; i <= Right.X + 1; i++)
                {
                    /** 
                     * Make the Boder Node :
                     * 1-traversable
                     * 2-Set Cost to Minimum Cost
                    * */
                    mapR[i, (int)Left.Y - 1].value = "B";
                    mapR[i, (int)Left.Y - 1].isBorder = true;
                    Point position = new Point(i, (int)Left.Y - 1);
                    /** Add Node to Structure to Reusing in Define Cut Size 
                        this is A further Improvement By Obada
                     * */
                    DownSide.Add(position);
                    //Add nodes to  General list to View the Boder
                    BorderList.Add(position);

                }

                //Build Up Side Border
                for (int i = (int)Left.X - 1; i <= Right.X + 1; i++)
                {

                    mapR[i, (int)Right.Y + 1].value = "B";
                    mapR[i, (int)Right.Y + 1].isBorder = true;
                    Point position = new Point(i, (int)Right.Y + 1);
                    UpSide.Add(position);
                    BorderList.Add(position);

                }

                //Build Left Side Boder
                for (int i = (int)Left.Y; i <= Right.Y; i++)
                {

                    mapR[(int)Left.X - 1, i].value = "B";
                    mapR[(int)Left.X - 1, i].isBorder = true;
                    Point position = new Point((int)Left.X - 1, i);
                    LeftSide.Add(position);
                    BorderList.Add(position);

                }

                //Build Right Side Boder
                for (int i = (int)Left.Y; i <= Right.Y; i++)
                {

                    mapR[(int)Right.X + 1, i].value = "B";
                    mapR[(int)Right.X + 1, i].isBorder = true;
                    Point position = new Point((int)Right.X + 1, i);
                    RightSide.Add(position);
                    BorderList.Add(position);

                }


                return mapR;
            }
        }

    }
}
