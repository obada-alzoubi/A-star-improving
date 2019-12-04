using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathFinder
{
    public class State
    {
        //Decalaration
        public double cost=0;
        public double heuristic=0;
        public double totalCost=0;
        public bool isBorder=false;
        public int processedCount=0;
        public int X=0;
        public int Y=0;
        public string value = "@";
        public int previous_X = -1;
        public int previous_Y = -1;
        

        //Constructor
        public State(int X, int Y, double cost, double heuristic, double totalCost, int processedCount, bool isBorder,string value ,int prevoius_X,int prevoius_Y)
        {
            this.X = X;
            this.Y = Y;
            this.cost = cost;
            this.heuristic = heuristic ;
            this.totalCost = totalCost ;
            this.processedCount = processedCount ;
            this.isBorder = isBorder ;
            this.value = value;
            this.previous_X = prevoius_X;
            this.previous_Y = prevoius_Y;
        }  

    }
}
