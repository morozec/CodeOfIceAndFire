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
    private static Random _rnd = new Random();

    class Point
    {
        public int X { get; set; }
        public int Y { get; set; }
        public int Owner { get; set; }
        public bool IsActive { get; set; }

        public Point(int x, int y, int owner, bool isActive)
        {
            X = x;
            Y = y;
            Owner = owner;
            IsActive = isActive;
        }
       
    }

    class Building : Point
    {
        public int BuildingType { get; set; }

        public Building(int x, int y, int owner, int buildingType) : base(x, y, owner, true)
        {
            BuildingType = buildingType;
        }
    }

    class Unit : Point
    {
        public int Id { get; set; }
        public int Level { get; set; }

        public Unit(int x, int y, int owner, int id, int level) : base(x, y, owner, true)
        {
            Id = id;
            Level = level;
        }
    }

    static void Main(string[] args)
    {
        string input;
        string[] inputs;

        var mineSpots = new List<Point>();

        input = Console.ReadLine(); Console.Error.WriteLine(input);
        int numberMineSpots = int.Parse(input);
        for (int i = 0; i < numberMineSpots; i++)
        {
            input = Console.ReadLine();Console.Error.WriteLine(input);
            inputs = input.Split(' ');
            int x = int.Parse(inputs[0]);
            int y = int.Parse(inputs[1]);
            mineSpots.Add(new Point(x, y,-1, true));
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


            var lines = new List<string>();
            for (int i = 0; i < 12; i++)
            {
                string line = Console.ReadLine(); Console.Error.WriteLine(line);
                lines.Add(line);
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
                else if (owner == 1)
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


            var map = GetMap(lines, myBuildings, myUnits, oppBuilding, oppUnits);
            var command = "";
            foreach (var myUnit in myUnits)
            {
                var movePoint = GetMovePoint(myUnit, map, isFire);

                if (movePoint != null)
                {
                    command += $"MOVE {myUnit.Id} {movePoint.X} {movePoint.Y};";
                    map[myUnit.Y][myUnit.X] = new Point(myUnit.X, myUnit.Y, 0, true);
                    map[movePoint.Y][movePoint.X] = new Unit(movePoint.X, movePoint.Y, myUnit.Owner, myUnit.Id, myUnit.Level);
                }
            }

            //var newLines = GetNewLines(lines, myUnits);

            //var mines = myBuildings.Where(b => b.BuildingType == 1).ToList();
            //while (gold >= GetMineCost(mines.Count))
            //{
            //    var bmp = GetBestMinePosition(newLines, mineSpots, mines, myBuildings.Single(b => b.BuildingType == 0), myUnits);
            //    if (bmp != null)
            //    {
            //        command += $"BUILD MINE {bmp.X} {bmp.Y};";
            //        mines.Add(new Building(bmp.X, bmp.Y, 0, 1));
            //        gold -= GetMineCost(mines.Count);
            //    }
            //    else
            //        break;
            //}
            

            UpdateMap(map, myBuildings.Single(b => b.BuildingType == 0));

            while (gold >= RecruitmentCost)
            {
                var recruitmentPoints = GetRecruitmentPoints(map);
                var bestRecruitmentPoint = GetBestRecruitmentPoint(recruitmentPoints, oppBuilding.Single(b => b.BuildingType == 0));
                if (bestRecruitmentPoint != null)
                {
                    command += $"TRAIN 1 {bestRecruitmentPoint.X} {bestRecruitmentPoint.Y};";
                    map[bestRecruitmentPoint.Y][bestRecruitmentPoint.X] =
                        new Unit(bestRecruitmentPoint.X, bestRecruitmentPoint.Y, 0, -1, 1);
                    gold -= RecruitmentCost;
                }
                else 
                    break;//иначе падаем в бесконечный цикл
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

    
    static IList<Point> GetRecruitmentPoints(IList<IList<Point>> map)
    {
        var recruitmentPoints = new List<Point>();
        for (var i = 0; i < map.Count; ++i)
        {
            for (var j = 0; j < map[i].Count; ++j)
            {
                var point = map[i][j];
                if (point == null) continue;
                if (point.Owner == 0)//my point
                {
                    if (point is Building pointBuilding)
                        continue;
                    if (point is Unit pointUnit)
                        continue;
                    if (point.IsActive)//is active
                        recruitmentPoints.Add(point);
                }
                else
                {
                    var isBorderPoint = i > 0 && map[i - 1][j] != null && map[i - 1][j].Owner == 0 && map[i-1][j].IsActive ||
                                        i < map.Count - 1 && map[i + 1][j] != null && map[i + 1][j].Owner == 0 && map[i+1][j].IsActive ||
                                        j > 0 && map[i][j - 1] != null && map[i][j - 1].Owner == 0 && map[i][j-1].IsActive ||
                                        j < map[i].Count - 1 && map[i][j + 1] != null && map[i][j + 1].Owner == 0 && map[i][j+1].IsActive;

                    if (!isBorderPoint)
                        continue;

                    if (point.Owner == 1)//opp point
                    {
                        if (point is Building pointBuilding)
                        {
                            if (pointBuilding.BuildingType == 0 || pointBuilding.BuildingType == 1) //base or mine
                                recruitmentPoints.Add(point);
                            continue;//TODO: строить на чужих башнях
                        }

                        if (point is Unit pointUnit)
                        {
                            continue;//TODO: строить на чужих солдатах
                        }

                        recruitmentPoints.Add(point);
                    }
                    else //neutral
                    {
                        recruitmentPoints.Add(point);
                    }
                }
            }
        }

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

    static Point GetMovePoint(Unit unit, IList<IList<Point>> map, bool isFire)
    {
        if (_rnd.NextDouble() > 0.5)
        {
            return GetHorizontalMove(unit, map, isFire) ??
                   GetVerticalMove(unit, map, isFire);
        }

        return GetVerticalMove(unit, map, isFire) ??
               GetHorizontalMove(unit, map, isFire);

    }

    static Point GetMapMovePoint(Unit unit, Point point)
    {
        if (point is Building pointBuilding)
        {
            if (pointBuilding.BuildingType == 0 || pointBuilding.BuildingType == 1) //base or mine
                return point;
            //tower
            if (pointBuilding.Owner == 0)//my tower
                return point;
            //opp tower
            if (unit.Level == 3)
                return point;
        }
        else if (point is Unit pointUnit)
        {
            if (pointUnit.Owner != 0 && (unit.Level == 3 || unit.Level > pointUnit.Level))
                return point;
        }
        else //нейтральная точка
        {
            return point;
        }

        return null;
    }

    static Point GetHorizontalMove(Unit unit, IList<IList<Point>> map, bool isFire)
    {
        if (isFire)
        {
            if (unit.X < map[unit.Y].Count - 1 && map[unit.Y][unit.X + 1] != null)
            {
                var mapMovePoint = GetMapMovePoint(unit, map[unit.Y][unit.X + 1]);
                if (mapMovePoint != null)
                    return mapMovePoint;
            }
        }
        else
        {
            if (unit.X > 0 && map[unit.Y][unit.X - 1] != null)
            {
                var mapMovePoint = GetMapMovePoint(unit, map[unit.Y][unit.X - 1]);
                if (mapMovePoint != null)
                    return mapMovePoint;
            }
        }

        return null;
    }

    static Point GetVerticalMove(Unit unit, IList<IList<Point>> map, bool isFire)
    {
        if (isFire)
        {
            if (unit.Y < map.Count - 1 && map[unit.Y + 1][unit.X] != null)
            {
                var mapMovePoint = GetMapMovePoint(unit, map[unit.Y + 1][unit.X]);
                if (mapMovePoint != null)
                    return mapMovePoint;
            }
        }
        else
        {
            if (unit.Y > 0 && map[unit.Y - 1][unit.X] != null)
            {
                var mapMovePoint = GetMapMovePoint(unit, map[unit.Y - 1][unit.X]);
                if (mapMovePoint != null)
                    return mapMovePoint;
            }
        }

        return null;
    }

    static IList<IList<Point>> GetMap(IList<string> lines, IList<Building> myBuildings, IList<Unit> myUnits,
        IList<Building> oppBuildings, IList<Unit> oppUnits)
    {
        var map = new List<IList<Point>>();
        for (var i = 0; i < lines.Count; ++i)
        {
            var mapLine = new List<Point>();
            for (var j = 0; j < lines[i].Length; ++j)
            {
                Point point = null;
                switch (lines[i][j])
                {
                    case 'o':
                        point = new Point(j, i, 0, false);
                        break;
                    case 'O':
                        point = new Point(j, i, 0, true);
                        break;
                    case 'x':
                        point = new Point(j, i, 1, false);
                        break;
                    case 'X':
                        point = new Point(j, i, 1, true);
                        break;
                    case '.':
                        point = new Point(j, i, -1, true);
                        break;
                }

                mapLine.Add(point);
            }
            map.Add(mapLine);
        }

        foreach (var b in myBuildings)
        {
            map[b.Y][b.X] = b;
        }

        foreach (var u in myUnits)
        {
            map[u.Y][u.X] = u;
        }

        foreach (var b in oppBuildings)
        {
            map[b.Y][b.X] = b;
        }

        foreach (var u in oppUnits)
        {
            map[u.Y][u.X] = u;
        }

        return map;
    }

    static void UpdateMap(IList<IList<Point>> map, Building myBase)
    {
        //BFS
        var queue = new Queue<Point>();
        queue.Enqueue(myBase);
        var discoveredPoints = new HashSet<Point>(){myBase};

        while (queue.Count > 0)
        {
            var p = queue.Dequeue();
            if (p.Owner == 0 && !p.IsActive)
            {
                p.IsActive = true;
                Console.Error.WriteLine($"POINT {p.X} {p.Y} IS ACTIVATED");
            }

            if (p.Y > 0)
            {
                var nP = map[p.Y - 1][p.X];
                if (nP != null && nP.Owner == 0 && !discoveredPoints.Contains(nP))
                {
                    discoveredPoints.Add(nP);
                    queue.Enqueue(nP);
                }
            }
            if (p.Y < map.Count - 1)
            {
                var nP = map[p.Y + 1][p.X];
                if (nP != null && nP.Owner == 0 && !discoveredPoints.Contains(nP))
                {
                    discoveredPoints.Add(nP);
                    queue.Enqueue(nP);
                }
            }

            if (p.X > 0)
            {
                var nP = map[p.Y][p.X - 1];
                if (nP != null && nP.Owner == 0 && !discoveredPoints.Contains(nP))
                {
                    discoveredPoints.Add(nP);
                    queue.Enqueue(nP);
                }
            }

            if (p.X < map[p.Y].Count - 1)
            {
                var nP = map[p.Y][p.X + 1];
                if (nP != null && nP.Owner == 0 && !discoveredPoints.Contains(nP))
                {
                    discoveredPoints.Add(nP);
                    queue.Enqueue(nP);
                }
            }
        }
    }
    

    static int GetMineCost(int mines)
    {
        return 20 + 4 * mines;
    }
   
}