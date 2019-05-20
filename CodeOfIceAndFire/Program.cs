using System;
using System.Linq;
using System.IO;
using System.Text;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using AStar;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
 

namespace AStar
{
    public abstract class Point : IComparable<Point>
    {
        public double G { get; set; }
        public double H { get; set; }

        public double F
        {
            get
            {
                return G + H;
            }
        }

        public Point CameFromPoint { get; set; }

        public abstract IEnumerable<Point> GetNeighbors(IEnumerable<Point> points);

        public abstract double GetHeuristicCost(Point goal);

        public abstract double GetCost(Point goal);

        public int CompareTo(Point other)
        {
            return -F.CompareTo(other.F);
        }
    }

    public class AStarPoint : Point
    {
        public int X { get; set; }
        public int Y { get; set; }
        public IEnumerable<AStarPoint> Neighbors { get; set; }
        public int Weight { get; set; }
        public int Owner { get; set; }

        public override IEnumerable<Point> GetNeighbors(IEnumerable<Point> points)
        {
            return Neighbors;
        }

        public override double GetHeuristicCost(Point goal)
        {
            var aStarGoal = goal as AStarPoint;
            return Player.GetManhDist(X, Y, aStarGoal.X, aStarGoal.Y);
        }

        public override double GetCost(Point goal)
        {
            var aStarGoal = goal as AStarPoint;
            var cost = Weight + Player.GetManhDist(X, Y, aStarGoal.X, aStarGoal.Y) * 10 * aStarGoal.Weight;
            if (aStarGoal.Owner == 1)
                cost -= 2;
            else if (aStarGoal.Owner == -1)
                cost -= 1;
            return cost;
        }

        public override string ToString()
        {
            return $"{X} {Y}";
        }
    }


    /// <summary>
    /// Матрица распростарения
    /// </summary>
    public class ExpansionMatrixConteiner
    {
        /// <summary>
        /// Стоимости прохода до точек сети
        /// </summary>
        public IDictionary<Point, double> ExpansionMatrix { get; set; }


        public Point RealGoalPoint { get; set; }

        ///// <summary>
        ///// Оптимальные пути прохода до точек сети
        ///// </summary>
        //public IDictionary<Point, IList<Point>> Path { get; set; } 
    }

    public static class Calculator
    {
        /// <summary>
        /// Расчет матрицы распространения
        /// </summary>
        /// <param name="start">Точка, для которой рассчитывается матрица распространения</param>
        /// <param name="goal">Целевая точка. Если null, то матрица распространения рассчитывается от стартовой точки до всех остальных точек сети</param>
        /// <param name="allPoints">Все точки сети</param>
        /// <returns>Матрица распространения</returns>
        private static ExpansionMatrixConteiner GetExpansionMatrix(Point start, Point goal, IEnumerable<Point> allPoints)
        {
            foreach (var point in allPoints)
            {
                point.CameFromPoint = null;
            }

            var emc = new ExpansionMatrixConteiner
            {
                ExpansionMatrix = new Dictionary<Point, double>(),
                //Path =  new Dictionary<Point, IList<Point>>()
            };

            var closedSet = new HashSet<Point>();
            var openSet = new HashSet<Point> { start };

            start.G = 0d;
            start.H = goal == null ? 0d : start.GetHeuristicCost(goal);

            var pathFound = false;

            while (openSet.Count > 0)
            {
                var x = GetPointWithMinF(openSet);

                if (goal != null && x == goal)
                {
                    pathFound = true;
                    emc.RealGoalPoint = goal;
                    break;
                }
                openSet.Remove(x);
                closedSet.Add(x);
                emc.ExpansionMatrix.Add(x, x.G);
                //emc.Path.Add(x, ReconstructPath(x));

                var neighbors = x.GetNeighbors(allPoints);
                foreach (var y in neighbors)
                {
                    if (closedSet.Contains(y)) continue;

                    var tentativeGScore = x.G + x.GetCost(y);
                    bool tentativeIsBetter;

                    if (!openSet.Contains(y))
                    {
                        openSet.Add(y);
                        tentativeIsBetter = true;
                    }
                    else
                    {
                        tentativeIsBetter = tentativeGScore < y.G;
                    }

                    if (tentativeIsBetter)
                    {
                        y.CameFromPoint = x;
                        y.G = tentativeGScore;
                        y.H = goal == null ? 0d : y.GetHeuristicCost(goal);
                    }
                }
            }

            if (goal != null && !pathFound) throw new Exception("Путь до конечной точки не найден");


            return emc;
        }

        /// <summary>
        /// Расчет оптимального пути до целевой точки
        /// </summary>
        /// <param name="start">Стартовая точка пути</param>
        /// <param name="goal">Целевая точка пути</param>
        /// <param name="allPoints">Все точки сети</param>
        /// <returns>Оптимальный путь от стартовой точки до целевой</returns>
        public static IList<Point> GetPath(Point start, Point goal, IEnumerable<Point> allPoints)
        {
            var emc = GetExpansionMatrix(start, goal, allPoints);
            return ReconstructPath(emc.RealGoalPoint);
        }

        /// <summary>
        /// Получение матриц распространения для набора стартовых точек
        /// </summary>
        /// <param name="startPoints">Набор стартовых точек</param>
        /// <param name="allPoints">Все точки сети</param>
        /// <returns>Матрицы распространения для стартовых точек</returns>
        private static IDictionary<Point, ExpansionMatrixConteiner> GetExpansionMatrices(
            IEnumerable<Point> startPoints, IEnumerable<Point> allPoints)
        {
            var result = new Dictionary<Point, ExpansionMatrixConteiner>();
            foreach (var startPoint in startPoints)
            {
                result.Add(startPoint, GetExpansionMatrix(startPoint, null, allPoints));
            }
            return result;
        }

        /// <summary>
        /// Получение точки с минимальным значением после суммирования матриц распространения
        /// </summary>
        /// <param name="expansionMatrices">Матрицы распространения</param>
        /// <param name="allPoints">Все точки сети</param>
        /// <returns>Точка с минимальной суммой</returns>
        private static Point GetMinCostPoint(IDictionary<Point, ExpansionMatrixConteiner> expansionMatrices, IEnumerable<Point> allPoints)
        {

            var summCosts = new Dictionary<Point, double>();
            foreach (var matrixPoint in allPoints)
            {
                summCosts.Add(matrixPoint, 0d);
                foreach (var startPoint in expansionMatrices.Keys)
                {
                    summCosts[matrixPoint] += expansionMatrices[startPoint].ExpansionMatrix[matrixPoint];
                }
            }

            Point cps = null;
            var summCost = double.MaxValue;
            foreach (var matrixPoint in summCosts.Keys)
            {
                if (summCosts[matrixPoint] < summCost)
                {
                    cps = matrixPoint;
                    summCost = summCosts[matrixPoint];
                }
            }

            return cps;
        }

        /// <summary>
        /// Получение точки с минимальной стомостью прохода до (и от) целевой 
        /// </summary>
        /// <param name="expansionMatrices">Матрицы распространения</param>
        /// <param name="notTraversedStartPoints">Список непройденных точек. Среди них будет проводиться поиск оптимальной</param>
        /// <param name="collectionPoint">Целевая точка</param>
        /// <returns>Точка с минимальной стомостью прохода</returns>
        private static Point GetNearestPoint(
            IDictionary<Point, ExpansionMatrixConteiner> expansionMatrices,
            IEnumerable<Point> notTraversedStartPoints,
            Point collectionPoint)
        {
            Point nearestPoint = null;
            var minCost = double.MaxValue;

            foreach (var point in notTraversedStartPoints)
            {
                if (expansionMatrices[point].ExpansionMatrix[collectionPoint] < minCost)
                {
                    nearestPoint = point;
                    minCost = expansionMatrices[point].ExpansionMatrix[collectionPoint];
                }
            }

            return nearestPoint;
        }


        /// <summary>
        /// Поиск точки с минимальной эврестической функцией (F)
        /// </summary>
        /// <param name="points">Список точек</param>
        /// <returns>Точка с минимальной эврестической функцией</returns>
        private static Point GetPointWithMinF(HashSet<Point> points)
        {
            if (!points.Any())
            {
                throw new Exception("Пустой список точек");
            }
            var minF = double.MaxValue;
            Point resultPoint = null;
            foreach (var point in points)
            {
                if (point.F < minF)
                {
                    minF = point.F;
                    resultPoint = point;
                }
            }

            return resultPoint;
        }

        /// <summary>
        /// Восстановление оптимального пути
        /// </summary>
        /// <param name="goal">Целевая точка</param>
        /// <returns>Оптимальный путь до целевой точки</returns>
        private static IList<Point> ReconstructPath(Point goal)
        {
            var resultList = new List<Point>();

            var currentPoint = goal;

            while (currentPoint != null)
            {
                resultList.Add(currentPoint);
                currentPoint = currentPoint.CameFromPoint;
            }

            resultList.Reverse();

            return resultList;
        }

        private static IList<Point> ReconstructPath(Point goal, ExpansionMatrixConteiner expansionMatrixConteiner, IEnumerable<Point> allPoints)
        {
            var path = new List<Point>() { goal };
            var currentPoint = goal;
            while (expansionMatrixConteiner.ExpansionMatrix[currentPoint] > 0)
            {
                Point closestNeighbour = null;
                var minCost = double.MaxValue;
                foreach (var neihgbour in currentPoint.GetNeighbors(allPoints))
                {
                    if (expansionMatrixConteiner.ExpansionMatrix[neihgbour] < minCost)
                    {
                        minCost = expansionMatrixConteiner.ExpansionMatrix[neihgbour];
                        closestNeighbour = neihgbour;
                    }
                }
                currentPoint = closestNeighbour;
                path.Add(closestNeighbour);
            }

            return path;
        }
    }
}


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

            var table = new AStarPoint[Size, Size];
            var allPoints = new List<AStarPoint>();

            var lines = new List<string>();
            for (int i = 0; i < Size; i++)
            {
                string line = Console.ReadLine(); Console.Error.WriteLine(line);

                for (var j = 0; j < line.Length; ++j)
                {
                    var owner = -1;
                    if (line[j] == 'o' || line[j] == 'O') owner = 0;
                    else if (line[j] == 'x' || line[j] == 'X') owner = 1;
                    var aStarPoint = new AStarPoint()
                        {X = j, Y = i, Weight = line[j] == '#' ? 10000 : 1, Owner = owner};
                    table[i, j] = aStarPoint;
                    allPoints.Add(aStarPoint);
                }
                lines.Add(line);
            }

            for (var i = 0; i < Size; ++i)
            {
                for (var j = 0; j < Size; ++j)
                {
                    var p = table[i, j];
                    var neighbours = new List<AStarPoint>();
                    if (i > 0) neighbours.Add(table[i - 1, j]);
                    if (i < Size - 1) neighbours.Add(table[i + 1, j]);
                    if (j > 0) neighbours.Add(table[i, j - 1]);
                    if (j < Size - 1) neighbours.Add(table[i, j + 1]);
                    p.Neighbors = neighbours;
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
            var oppBase = oppBuilding.Single(b => b.BuildingType == 0);
            var command = "";

            var endPoint = table[oppBase.Y, oppBase.X];
            var paths = new List<Tuple<Unit, IList<AStar.Point>>>();

            foreach (var myUnit in myUnits)
            {
                var startPoint = table[myUnit.Y, myUnit.X];
                var path = AStar.Calculator.GetPath(startPoint, endPoint, allPoints);
                paths.Add(new Tuple<Unit, IList<AStar.Point>>(myUnit, path));
            }

            paths = paths.OrderBy(p => p.Item2.Count).ToList();
            foreach (var p in paths)
            {
                var myUnit = p.Item1;
                if (p.Item2.Count < 2) continue;
                var step = p.Item2[1] as AStarPoint;
                if (CanMove(myUnit, map[step.Y][step.X]))
                {
                    command += $"MOVE {myUnit.Id} {step.X} {step.Y};";
                    map[myUnit.Y][myUnit.X] = new Point(myUnit.X, myUnit.Y, 0, true);
                    map[step.Y][step.X] = new Unit(step.X, step.Y, myUnit.Owner, myUnit.Id, myUnit.Level);
                }
            }

            //var newLines = GetNewLines(lines, myUnits);

            
            

            UpdateMap(map, myBuildings.Single(b => b.BuildingType == 0));

            while (gold >= RecruitmentCost)
            {
                var recruitmentPoints = GetRecruitmentPoints(map);
                var bestRecruitmentPoint = GetBestRecruitmentPoint(recruitmentPoints, oppBase);
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

           
            var mines = myBuildings.Where(b => b.BuildingType == 1).ToList();
            while (gold >= GetMineCost(mines.Count))
            {
                var bmp = GetBestMinePosition(map, mineSpots, myBuildings.Single(b => b.BuildingType == 0));
                if (bmp != null)
                {
                    command += $"BUILD MINE {bmp.X} {bmp.Y};";
                    mines.Add(new Building(bmp.X, bmp.Y, 0, 1));
                    map[bmp.Y][bmp.X] = new Building(bmp.X, bmp.Y, 0, 1);
                    gold -= GetMineCost(mines.Count);
                }
                else
                    break;
            }
        

            Console.WriteLine(command != "" ? command : "WAIT");
        }
    }

    static int GetManhDist(Point p1, Point p2)
    {
        return GetManhDist(p1.X, p1.Y, p2.X, p2.Y);
    }

    public static int GetManhDist(int x1, int y1, int x2, int y2)
    {
        return Math.Abs(x1 - x2) + Math.Abs(y1 - y2);
    }

    static Point GetBestMinePosition(IList<IList<Point>> map, IList<Point> mineSpots,Building myBase)
    {
        Point bestMineSpot = null;
        int minDist = int.MaxValue;
        foreach (var ms in mineSpots)
        {
            var isOkPoint = map[ms.Y][ms.X] != null && !(map[ms.Y][ms.X] is Building) && !(map[ms.Y][ms.X] is Unit) &&
                             map[ms.Y][ms.X].Owner == 0 && map[ms.Y][ms.X].IsActive;
            if (!isOkPoint) continue;
            var dist = GetManhDist(ms, myBase);
            if (dist < minDist)
            {
                minDist = dist;
                bestMineSpot = ms;
            }
        }

        return bestMineSpot;
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
                    continue;
                    //if (point is Building pointBuilding)
                    //    continue;
                    //if (point is Unit pointUnit)
                    //    continue;
                    //if (point.IsActive)//is active
                    //    recruitmentPoints.Add(point);
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

    //static bool IsMyPoint(char c)
    //{
    //    return c == 'o' || c == 'O';
    //}

    //static Point GetMovePoint(Unit unit, IList<IList<Point>> map, bool isFire)
    //{
    //    if (_rnd.NextDouble() > 0.5)
    //    {
    //        return GetHorizontalMove(unit, map, isFire) ??
    //               GetVerticalMove(unit, map, isFire);
    //    }

    //    return GetVerticalMove(unit, map, isFire) ??
    //           GetHorizontalMove(unit, map, isFire);

    //}

    static bool CanMove(Unit unit, Point point)
    {
        if (point is Building pointBuilding)
        {
            if (pointBuilding.BuildingType == 0 || pointBuilding.BuildingType == 1) //base or mine
                return true;
            //tower
            if (pointBuilding.Owner == 0)//my tower
                return true;
            //opp tower
            if (unit.Level == 3)
                return true;
        }
        else if (point is Unit pointUnit)
        {
            if (pointUnit.Owner != 0 && (unit.Level == 3 || unit.Level > pointUnit.Level))
                return true;
        }
        else //нейтральная точка
        {
            return true;
        }

        return false;
    }

    //static Point GetHorizontalMove(Unit unit, IList<IList<Point>> map, bool isFire)
    //{
    //    if (isFire)
    //    {
    //        if (unit.X < map[unit.Y].Count - 1 && map[unit.Y][unit.X + 1] != null)
    //        {
    //            var mapMovePoint = GetMapMovePoint(unit, map[unit.Y][unit.X + 1]);
    //            if (mapMovePoint != null)
    //                return mapMovePoint;
    //        }
    //    }
    //    else
    //    {
    //        if (unit.X > 0 && map[unit.Y][unit.X - 1] != null)
    //        {
    //            var mapMovePoint = GetMapMovePoint(unit, map[unit.Y][unit.X - 1]);
    //            if (mapMovePoint != null)
    //                return mapMovePoint;
    //        }
    //    }

    //    return null;
    //}

    //static Point GetVerticalMove(Unit unit, IList<IList<Point>> map, bool isFire)
    //{
    //    if (isFire)
    //    {
    //        if (unit.Y < map.Count - 1 && map[unit.Y + 1][unit.X] != null)
    //        {
    //            var mapMovePoint = GetMapMovePoint(unit, map[unit.Y + 1][unit.X]);
    //            if (mapMovePoint != null)
    //                return mapMovePoint;
    //        }
    //    }
    //    else
    //    {
    //        if (unit.Y > 0 && map[unit.Y - 1][unit.X] != null)
    //        {
    //            var mapMovePoint = GetMapMovePoint(unit, map[unit.Y - 1][unit.X]);
    //            if (mapMovePoint != null)
    //                return mapMovePoint;
    //        }
    //    }

    //    return null;
    //}

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