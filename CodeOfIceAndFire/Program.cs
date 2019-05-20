using System;
using System.Linq;
using System.IO;
using System.Text;
using System.Collections;
using System.Collections.Generic;
using System.Net;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
class Player
{
    const int RecruitmentCost = 10;
    private const int Size = 12;

    class Point
    {
        public int X { get; set; }
        public int Y { get; set; }
        public int Owner { get; set; }

        public Point(int x, int y, int owner)
        {
            X = x;
            Y = y;
            Owner = owner;
        }
       
    }

    class Building : Point
    {
        public int BuildingType { get; set; }

        public Building(int x, int y, int owner, int buildingType) : base(x, y, owner)
        {
            BuildingType = buildingType;
        }
    }

    class Unit : Point
    {
        public int Id { get; set; }
        public int Level { get; set; }

        public Unit(int x, int y, int owner, int id, int level) : base(x, y, owner)
        {
            Id = id;
            Level = level;
        }
    }

    static void Main(string[] args)
    {
        string input;
        string[] inputs;

        input = Console.ReadLine(); Console.Error.WriteLine(input);
        int numberMineSpots = int.Parse(input);
        for (int i = 0; i < numberMineSpots; i++)
        {
            input = Console.ReadLine();Console.Error.WriteLine(input);
            inputs = input.Split(' ');
            int x = int.Parse(inputs[0]);
            int y = int.Parse(inputs[1]);
        }

        // game loop
        while (true)
        {
            var myUnits = new List<Unit>();
            var oppUnits = new List<Unit>();


            int gold = int.Parse(Console.ReadLine()); Console.Error.WriteLine(gold);
            int income = int.Parse(Console.ReadLine()); Console.Error.WriteLine(income);
            int opponentGold = int.Parse(Console.ReadLine()); Console.Error.WriteLine(opponentGold);
            int opponentIncome = int.Parse(Console.ReadLine()); Console.Error.WriteLine(opponentIncome);

            var myPoints = new List<Point>();
            //var nearNeutralPoints = new List<Point>();

            var lines = new List<string>();
            for (int i = 0; i < 12; i++)
            {
                string line = Console.ReadLine(); Console.Error.WriteLine(line);
                lines.Add(line);

                for (var j = 0; j < line.Length; ++j)
                {
                    if (IsMyPoint(line[j]))
                        myPoints.Add(new Point(j, i, 0));

                }
            }



            bool isFire = true;
            var myBuildings = new List<Building>();
            var oppBuilding = new List<Building>();

            input = Console.ReadLine(); Console.Error.WriteLine(input);
            int buildingCount = int.Parse(input);
            for (int i = 0; i < buildingCount; i++)
            {
                input = Console.ReadLine(); Console.Error.WriteLine(input);
                inputs = input.Split(' ');
                int owner = int.Parse(inputs[0]);
                int buildingType = int.Parse(inputs[1]);
                int x = int.Parse(inputs[2]);
                int y = int.Parse(inputs[3]);
                
                
                var someBuilding = new Building(x, y, owner, buildingType);
                if (owner == 0)
                {
                    myBuildings.Add(someBuilding);
                    if (buildingType == 0 && x != 0 && y != 0)
                        isFire = false;
                }
                else
                    oppBuilding.Add(someBuilding);
                
            }


            input = Console.ReadLine(); Console.Error.WriteLine(input);
            int unitCount = int.Parse(input);
            for (int i = 0; i < unitCount; i++)
            {
                input = Console.ReadLine(); Console.Error.WriteLine(input);
                inputs = input.Split(' ');
                int owner = int.Parse(inputs[0]);
                int unitId = int.Parse(inputs[1]);
                int level = int.Parse(inputs[2]);
                int x = int.Parse(inputs[3]);
                int y = int.Parse(inputs[4]);

                var unit = new Unit(x, y, owner, unitId, level);
                if (owner == 0)
                    myUnits.Add(unit);
                else
                    oppUnits.Add(unit);
            }

            var command = "";
            foreach (var myUnit in myUnits)
            {
                var movePoint = GetMovePoint(myUnit, lines, myBuildings, myUnits, isFire);

                if (movePoint != null)
                {
                    command += $"MOVE {myUnit.Id} {movePoint.X} {movePoint.Y};";
                    myUnit.X = movePoint.X;
                    myUnit.Y = movePoint.Y;
                }
            }

            var newLines = GetNewLines(lines, myUnits);
           

            if (gold > RecruitmentCost)
            {
                var nearNeutralPoints = GetNearNeutralPoints(newLines);
                var recruitmentPoints = GetRecruitmentPoints(myPoints, nearNeutralPoints, myBuildings, myUnits);


                var bestRecruitmentPoint = GetBestRecruitmentPoint(recruitmentPoints, oppBuilding.Single(b => b.BuildingType == 0));
                if (bestRecruitmentPoint != null)
                {
                    command += $"TRAIN 1 {bestRecruitmentPoint.X} {bestRecruitmentPoint.Y}";
                }
            }

            Console.WriteLine(command != "" ? command : "WAIT");
        }
    }

    static int GetManhDist(Point p1, Point p2)
    {
        return GetManhDist(p1.X, p1.Y, p2.X, p2.Y);
    }

    static int GetManhDist(int x1, int y1, int x2, int y2)
    {
        return Math.Abs(x1 - x2) + Math.Abs(y1 - y2);
    }

    static IList<Point> GetNearNeutralPoints(IList<string> lines)
    {
        var nearNeutralPoints = new List<Point>();
        for (var i = 0; i < lines.Count; ++i)
        {
            var line = lines[i];
            for (var j = 0; j < lines[i].Count(); ++j)
            {
                if (line[j] != '.') continue;
               
                if (j > 0 && IsMyPoint(line[j - 1]) ||
                        j < line.Length - 1 && IsMyPoint(line[j + 1]) ||
                        i > 0 && IsMyPoint(lines[i - 1][j]) ||
                        i < lines.Count - 1 && IsMyPoint(lines[i + 1][j]))
                {
                    nearNeutralPoints.Add(new Point(j, i, -1));
                    //Console.Error.WriteLine(j + " " + i);
                }
            }
        
        }

        return nearNeutralPoints;
    }

    static IList<Point> GetRecruitmentPoints(
        IList<Point> myPoints, IList<Point> nearNeutralPoints, IList<Building> myBuildings, IList<Unit> myUnits)
    {
        var recruitmentPoints = new List<Point>();
        foreach (var p in myPoints)
        {
            if (!myBuildings.Any(b => b.X == p.X && b.Y == p.Y) && !myUnits.Any(u => u.X == p.X && u.Y == p.Y))
                recruitmentPoints.Add(p);
        }
        recruitmentPoints.AddRange(nearNeutralPoints);
        return recruitmentPoints;
    }

    static Point GetBestRecruitmentPoint(IList<Point> recruitmentPoints, Point oppBase)
    {
        Point bestPoint = null;
        int minDist = int.MaxValue;

        foreach (var p in recruitmentPoints)
        {
            var dist = GetManhDist(p, oppBase);
            if (dist < minDist)
            {
                minDist = dist;
                bestPoint = p;
            }
        }

        return bestPoint;

    }

    static bool IsMyPoint(char c)
    {
        return c == 'o' || c == 'O';
    }

    static Point GetMovePoint(Unit unit, IList<string> lines, IList<Building> myBuildings, IList<Unit> myUnits, bool isFire)
    {
        if (isFire)
        {
            if (unit.X < lines[unit.Y].Length - 1)
            {
                if (lines[unit.Y][unit.X + 1] != '#' && !myBuildings.Any(b => b.X == unit.X + 1 && b.Y == unit.Y) &&
                    !myUnits.Any(u => u.X == unit.X + 1 && u.Y == unit.Y))
                    return new Point(unit.X + 1, unit.Y, -1);
            }

            if (unit.Y < lines.Count - 1)
            {
                if (lines[unit.Y + 1][unit.X] != '#' && !myBuildings.Any(b => b.X == unit.X && b.Y == unit.Y + 1) &&
                    !myUnits.Any(u => u.X == unit.X && u.Y == unit.Y + 1))
                    return new Point(unit.X, unit.Y + 1, -1);
            }
        }
        else
        {
            if (unit.X > 0)
            {
                if (lines[unit.Y][unit.X - 1] != '#' && !myBuildings.Any(b => b.X == unit.X - 1 && b.Y == unit.Y) &&
                    !myUnits.Any(u => u.X == unit.X - 1 && u.Y == unit.Y))
                    return new Point(unit.X - 1, unit.Y, -1);
            }

            if (unit.Y > 0)
            {
                if (lines[unit.Y - 1][unit.X] != '#' && !myBuildings.Any(b => b.X == unit.X && b.Y == unit.Y - 1) &&
                    !myUnits.Any(u => u.X == unit.X && u.Y == unit.Y - 1))
                    return new Point(unit.X, unit.Y - 1, -1);
            }
        }

        return null;
    }

    static IList<string> GetNewLines(IList<string> lines, IList<Unit> myUnits)
    {
        var newLines = new List<string>();
        for (var i = 0; i < lines.Count; ++i)
        {
            var newLine = "";
            for (var j = 0; j < lines[i].Length; ++j)
            {
                if (myUnits.Any(u => u.X == j && u.Y == i))
                    newLine += 'O';
                else
                    newLine += lines[i][j];
            }
            newLines.Add(newLine);
        }

        return newLines;
    }
}