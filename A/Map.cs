using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathFinder
{
    public class Map
    {
        //Decalration
        public string[] mapString;
        public int width = 0;
        public int height = 0;

        //Constructor
        public Map(string[] mapString,int width,int height)
        {
            this.mapString = mapString;
            this.width = width;
            this.height = height;

        }

        //Method to build the Map
        public State[,] MapBuild()
        {
            State[,] map=new State[height,width];
            //build the Map
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    map[i,j] = new State(i, j, 0, 0, 0, 0, false,mapString[i+4][j].ToString(),-1,-1);
                    if (i > 150 && i < 250)
                        if (j > 60 && j < 200)
                        {

                            map[i, j].cost = 1000 - (Math.Abs(193 - i) + Math.Abs(114 - j));
                           // Console.WriteLine(map[i, j].cost);
                        }
                }
            }

            return map;

        }


    }
}
