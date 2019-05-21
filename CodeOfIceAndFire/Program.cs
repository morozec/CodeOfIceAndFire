using System;
using System.Linq;
using System.IO;
using System.Text;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Runtime.Remoting.Channels;
using AStar;
using MapPoints;


/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
 

namespace AStar
{
    public abstract class APoint : IComparable<APoint>
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

        public APoint CameFromPoint { get; set; }

        public abstract IEnumerable<APoint> GetNeighbors(IEnumerable<APoint> points);

        public abstract double GetHeuristicCost(APoint goal);

        public abstract double GetCost(APoint goal, int unitLevel, IList<IList<Point>> map);

        public int CompareTo(APoint other)
        {
            return -F.CompareTo(other.F);
        }
    }

    public class AStarPoint : APoint
    {
        const int BIG_WEIGHT = 10000;

        public int X { get; set; }
        public int Y { get; set; }
        public IEnumerable<AStarPoint> Neighbors { get; set; }
        public int Weight { get; set; }
        public int Owner { get; set; }
        public bool IsMySolder { get; set; }

        public override IEnumerable<APoint> GetNeighbors(IEnumerable<APoint> points)
        {
            return Neighbors;
        }

        public override double GetHeuristicCost(APoint goal)
        {
            var aStarGoal = goal as AStarPoint;
            return Player.GetManhDist(X, Y, aStarGoal.X, aStarGoal.Y);
        }

        public override double GetCost(APoint goal, int unitLevel, IList<IList<Point>> map)
        {
            var aStarGoal = goal as AStarPoint;
            if (unitLevel < 3 && Player.IsTowerInfluenceCell(aStarGoal.X, aStarGoal.Y, map, 1))
                return BIG_WEIGHT;

            double cost = Weight + Player.GetManhDist(X, Y, aStarGoal.X, aStarGoal.Y) * aStarGoal.Weight;
            if (aStarGoal.Owner == 1)
                cost -= 0.1;
            if (aStarGoal.Owner == -1)
                cost -= 0.2;
            if (aStarGoal.IsMySolder)
                cost += 10;

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
        public IDictionary<APoint, double> ExpansionMatrix { get; set; }


        public APoint RealGoalPoint { get; set; }

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
        private static ExpansionMatrixConteiner GetExpansionMatrix(APoint start, APoint goal, IEnumerable<APoint> allPoints,
            int unitLevel, IList<IList<Point>> map )
        {
            foreach (var point in allPoints)
            {
                point.CameFromPoint = null;
            }

            var emc = new ExpansionMatrixConteiner
            {
                ExpansionMatrix = new Dictionary<APoint, double>(),
                //Path =  new Dictionary<Point, IList<Point>>()
            };

            var closedSet = new HashSet<APoint>();
            var openSet = new HashSet<APoint> { start };

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

                    var tentativeGScore = x.G + x.GetCost(y,unitLevel, map);
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
        public static IList<APoint> GetPath(APoint start, APoint goal, IEnumerable<APoint> allPoints, int unitLevel, IList<IList<Point>> map)
        {
            var emc = GetExpansionMatrix(start, goal, allPoints, unitLevel, map);
            return ReconstructPath(emc.RealGoalPoint);
        }
        

        /// <summary>
        /// Получение точки с минимальным значением после суммирования матриц распространения
        /// </summary>
        /// <param name="expansionMatrices">Матрицы распространения</param>
        /// <param name="allPoints">Все точки сети</param>
        /// <returns>Точка с минимальной суммой</returns>
        private static APoint GetMinCostPoint(IDictionary<APoint, ExpansionMatrixConteiner> expansionMatrices, IEnumerable<APoint> allPoints)
        {

            var summCosts = new Dictionary<APoint, double>();
            foreach (var matrixPoint in allPoints)
            {
                summCosts.Add(matrixPoint, 0d);
                foreach (var startPoint in expansionMatrices.Keys)
                {
                    summCosts[matrixPoint] += expansionMatrices[startPoint].ExpansionMatrix[matrixPoint];
                }
            }

            APoint cps = null;
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
        private static APoint GetNearestPoint(
            IDictionary<APoint, ExpansionMatrixConteiner> expansionMatrices,
            IEnumerable<APoint> notTraversedStartPoints,
            APoint collectionPoint)
        {
            APoint nearestPoint = null;
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
        private static APoint GetPointWithMinF(HashSet<APoint> points)
        {
            if (!points.Any())
            {
                throw new Exception("Пустой список точек");
            }
            var minF = double.MaxValue;
            APoint resultPoint = null;
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
        private static IList<APoint> ReconstructPath(APoint goal)
        {
            var resultList = new List<APoint>();

            var currentPoint = goal;

            while (currentPoint != null)
            {
                resultList.Add(currentPoint);
                currentPoint = currentPoint.CameFromPoint;
            }

            resultList.Reverse();

            return resultList;
        }

        private static IList<APoint> ReconstructPath(APoint goal, ExpansionMatrixConteiner expansionMatrixConteiner, IEnumerable<APoint> allPoints)
        {
            var path = new List<APoint>() { goal };
            var currentPoint = goal;
            while (expansionMatrixConteiner.ExpansionMatrix[currentPoint] > 0)
            {
                APoint closestNeighbour = null;
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


namespace MapPoints
{
    public class Point
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
}


class Player
{
    const int BIG_WEIGHT = 10000;

    const int RecruitmentCost1 = 10;
    const int RecruitmentCost2 = 20;
    const int RecruitmentCost3 = 30;
    const int TowerCost = 15;
    private const int Size = 12;
    private static Random _rnd = new Random();

    

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
                        {X = j, Y = i, Weight = line[j] == '#' ? BIG_WEIGHT : 1, Owner = owner, IsMySolder = false};
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
                {
                    myUnits.Add(unit);
                    table[y, x].IsMySolder = true;
                }
                else
                    oppUnits.Add(unit);
            }


            var map = GetMap(lines, myBuildings, myUnits, oppBuilding, oppUnits);
            var myBase = myBuildings.Single(b => b.BuildingType == 0);
            var oppBase = oppBuilding.Single(b => b.BuildingType == 0);
            var command = "";

            var endPoint = table[oppBase.Y, oppBase.X];

            myUnits = myUnits.OrderBy(u => GetManhDist(u, oppBase)).ToList();
            var noWayUnits = new List<Unit>();
            foreach (var myUnit in myUnits)
            {
                var startPoint = table[myUnit.Y, myUnit.X];
                var path = AStar.Calculator.GetPath(startPoint, endPoint, allPoints, myUnit.Level, map);
                if (path.Count < 2) continue;
 
                var isMySolder = false;
                for (var i = 1; i < path.Count; ++i)
                {
                    if ((path[i] as AStarPoint).IsMySolder)
                    {
                        isMySolder = true;
                        break;
                    }
                }

                if (isMySolder)
                {
                    noWayUnits.Add(myUnit);
                    continue;
                }

                var step =path[1] as AStarPoint;
                if (CanMove(myUnit, map[step.Y][step.X], map))
                {
                    command += $"MOVE {myUnit.Id} {step.X} {step.Y};";
                    map[myUnit.Y][myUnit.X] = new Point(myUnit.X, myUnit.Y, 0, true);
                    map[step.Y][step.X] = new Unit(step.X, step.Y, myUnit.Owner, myUnit.Id, myUnit.Level);

                    table[step.Y, step.X].Owner = 0;
                    table[myUnit.Y, myUnit.X].IsMySolder = false;
                    table[step.Y, step.X].IsMySolder = true;
                }

            }

           
            //paths = paths.OrderBy(p => p.Item2.Count).ToList();
            //foreach (var p in paths)
            //{
            //    var myUnit = p.Item1;
                
            //    var step = p.Item2[1] as AStarPoint;
            //    if (CanMove(myUnit, map[step.Y][step.X], map))
            //    {
            //        command += $"MOVE {myUnit.Id} {step.X} {step.Y};";
            //        map[myUnit.Y][myUnit.X] = new Point(myUnit.X, myUnit.Y, 0, true);
            //        map[step.Y][step.X] = new Unit(step.X, step.Y, myUnit.Owner, myUnit.Id, myUnit.Level);

            //        table[step.Y, step.X].Owner = 0;
            //        table[myUnit.Y, myUnit.X].IsMySolder = false;
            //        table[step.Y, step.X].IsMySolder = true;
            //    }
            //}


            //если солдат не может дойти до чужой базы, он идет захватывать свободные точки
            var canCapturePoints = new List<Point>();
            foreach (var line in map)
            {
                foreach (var point in line)
                {
                    if (point != null && point.Owner != 0)
                        canCapturePoints.Add(point);
                }
            }


            while (noWayUnits.Any() && canCapturePoints.Any())
            {
                var minDist = int.MaxValue;
                Unit bestUnit = null;
                Point bestCcp = null;

                foreach (var ccp in canCapturePoints)
                {
                    foreach (var unit in noWayUnits)
                    {
                        var dist = GetManhDist(unit, ccp);
                        if (dist < minDist)
                        {
                            minDist = dist;
                            bestUnit = unit;
                            bestCcp = ccp;
                        }
                    }
                }

                if (bestUnit == null || bestCcp == null)
                    break;

                var bestPath = Calculator.GetPath(table[bestUnit.Y, bestUnit.X],
                    table[bestCcp.Y, bestCcp.X],
                    allPoints,
                    bestUnit.Level,
                    map);
                
                var bestStep = bestPath[1] as AStarPoint;
                if (CanMove(bestUnit, map[bestStep.Y][bestStep.X], map))
                {
                    command += $"MOVE {bestUnit.Id} {bestStep.X} {bestStep.Y};";
                    map[bestUnit.Y][bestUnit.X] = new Point(bestUnit.X, bestUnit.Y, 0, true);
                    map[bestStep.Y][bestStep.X] =
                        new Unit(bestStep.X, bestStep.Y, bestUnit.Owner, bestUnit.Id, bestUnit.Level);

                    table[bestStep.Y, bestStep.X].Owner = 0;
                    table[bestUnit.Y, bestUnit.X].IsMySolder = false;
                    table[bestStep.Y, bestStep.X].IsMySolder = true;
                }

                canCapturePoints.Remove(bestCcp);
                noWayUnits.Remove(bestUnit);

            }



            UpdateMap(map, myBuildings.Single(b => b.BuildingType == 0));

            while (gold >= RecruitmentCost1)
            {
                var recruitmentPoints = GetRecruitmentPoints(map);
                var bestRecruitmentPoint =
                    GetBestRecruitmentPoint(recruitmentPoints, myBase, oppBase, table, allPoints, map, mineSpots,gold, income);
                if (bestRecruitmentPoint != null)
                {
                    if (bestRecruitmentPoint is Unit bestRecruitmentUnit)
                    {

                        command +=
                            $"TRAIN {bestRecruitmentUnit.Level} {bestRecruitmentUnit.X} {bestRecruitmentUnit.Y};";
                        map[bestRecruitmentUnit.Y][bestRecruitmentUnit.X] = bestRecruitmentUnit;
                        int cost = GetUnitCost(bestRecruitmentUnit.Level);
                        gold -= cost;
                    }
                    else if (bestRecruitmentPoint is Building bestRecruitmentBuilding)
                    {
                        command += $"BUILD TOWER {bestRecruitmentBuilding.X} {bestRecruitmentBuilding.Y};";
                        map[bestRecruitmentBuilding.Y][bestRecruitmentBuilding.X] = bestRecruitmentBuilding;
                        gold -= TowerCost;
                    }
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

    static int GetUnitCost(int level)
    {
        return level == 3 ? RecruitmentCost3 : level == 2 ? RecruitmentCost2 : RecruitmentCost1;
    }

    static int GetUnitUpkeep(int level)
    {
        return level == 3 ? 20 : level == 2 ? 4 : 1;
    }

    static Point GetBestRecruitmentPoint(IList<Point> recruitmentPoints, Building myBase, Building oppBase,
        AStarPoint[,] table, IList<AStarPoint> allPoints,
        IList<IList<Point>> map, IList<Point> mines,
        int gold, int income)
    {
        Unit bestUnit = null;
        int maxKillCount = 0;
        int minOppBaseDist = int.MaxValue;



        var end = table[oppBase.Y, oppBase.X];

        foreach (var p in recruitmentPoints)
        {
            int level = 1;

            if (IsTowerInfluenceCell(p.X, p.Y, map, 1))
            {
                if (gold < RecruitmentCost3) continue;
                level = 3;
            }
            else
            {
                if (IsTowerCell(p.X, p.Y, map, 1))
                {
                    if (gold < RecruitmentCost3) continue;
                    level = 3;
                }
                else if (p is Unit pUnit)
                {
                    switch (pUnit.Level)
                    {
                        case 3:
                            if (gold < RecruitmentCost3)
                                continue;
                            level = 3;
                            break;
                        case 2:
                            if (gold < RecruitmentCost3)
                                continue;
                            level = 3;
                            break;
                        case 1:
                            if (gold < RecruitmentCost2)
                                continue;
                            level = 2;
                            break;
                    }
                }
            }

            var unitUpkeep = GetUnitUpkeep(level);
            if (unitUpkeep > income)
                continue;

            var killCount = GetKillCount(p, map, oppBase);
            if (killCount < maxKillCount) continue;




            var minMyUnitsDist = int.MaxValue;
            for (var i = 0; i < map.Count; ++i)
            {
                for (var j = 0; j < map.Count; ++j)
                {
                    if (map[i][j] == null || map[i][j].Owner != 0 || !(map[i][j] is Unit)) continue;
                    var dist = GetManhDist(map[i][j], p);
                    if (dist < minMyUnitsDist)
                        minMyUnitsDist = dist;
                }
            }

            var start = table[p.Y, p.X];
            var path = Calculator.GetPath(start, end, allPoints, 3, map);

            if (killCount > maxKillCount ||
                killCount == maxKillCount && path.Count < minOppBaseDist)
            {
                maxKillCount = killCount;
                minOppBaseDist = path.Count;
                bestUnit = new Unit(p.X, p.Y, 0, -1, level);
            }
        }

        if (maxKillCount > 0 || gold < TowerCost)
            return bestUnit;

        Unit mostDangerousOppUnit = null;
        int minMyBaseDist = int.MaxValue;
        for (var i = 0; i < map.Count; ++i)
        {
            for (var j = 0; j < map[i].Count; ++j)
            {
                var p = map[i][j];
                if (p == null || p.Owner != 1 || !(p is Unit)) continue;
                var dist = GetManhDist(p, myBase);
                if (dist < minMyBaseDist)
                {
                    minMyBaseDist = dist;
                    mostDangerousOppUnit = p as Unit;
                }
            }
        }

        if (mostDangerousOppUnit == null)
            return bestUnit;


        int maxMyPointsCover = 0;
        int maxMyUnitsCover = 0;
        int minMyBaseDistCover = int.MaxValue;
        Building tower = null;


        var neighbours = GetMapNeighbours(map, mostDangerousOppUnit, true);
        foreach (var n in neighbours)
        {
            if (n == null || n.Owner != 0 || !n.IsActive ||
                (n is Unit || n is Building))
                continue;
            if (mines.Any(m => m.X == n.X && m.Y == n.Y)) continue;
            
           
            
            var myPointsCover = 0;
            var myUnitsCover = 0;

            var hasMyTowersNear = false;

            var nNeighbours = GetMapNeighbours(map, n, false);
            foreach (var nn in nNeighbours)
            {
                if (nn == null) continue;

                //no my towers near
                if (IsTowerInfluenceCell(nn.X, nn.Y, map, 0))
                {
                    hasMyTowersNear = true;
                    break;
                }

                if (nn.Owner == 0)
                {
                    myPointsCover++;
                    if (nn is Unit)
                        myUnitsCover++;
                }
            }

            if (hasMyTowersNear) continue;

            if (myPointsCover > maxMyPointsCover || myPointsCover == maxMyPointsCover && myUnitsCover > maxMyUnitsCover ||
                myPointsCover == maxMyPointsCover && myUnitsCover == maxMyUnitsCover && GetManhDist(n, myBase) < minMyBaseDistCover)
            {
                maxMyPointsCover = myPointsCover;
                maxMyUnitsCover = myUnitsCover;
                minMyBaseDistCover = GetManhDist(n, myBase);
                tower = new Building(n.X, n.Y, 1, 2);
            }

        }

        if (tower != null)
            return tower;
        return bestUnit;

    }


    static int GetKillCount(Point step, IList<IList<Point>> map, Building oppBase)
    {
        var oppUnitsCount = 0;
        for (var i = 0; i < map.Count; ++i)
        {
            for (var j = 0; j < map[i].Count; ++j)
            {
                if (map[i][j] is Unit mapUnit && mapUnit.Owner == 1)
                    oppUnitsCount++;
            }
        }

        var mapPoint = map[step.Y][step.X];
        map[step.Y][step.X] = new Unit(step.X, step.Y, 0, -1, 1);

        //BFS
        var newOppUnitsCount = 0;


        var queue = new Queue<Point>();
        queue.Enqueue(oppBase);
        var discoveredPoints = new HashSet<Point>() { oppBase };

        while (queue.Count > 0)
        {
            var p = queue.Dequeue();
            if (p is Unit pUnit && pUnit.Owner == 1)
                newOppUnitsCount++;
           
            if (p.Y > 0)
            {
                var nP = map[p.Y - 1][p.X];
                if (nP != null && nP.Owner == 1 && !discoveredPoints.Contains(nP))
                {
                    discoveredPoints.Add(nP);
                    queue.Enqueue(nP);
                }
            }
            if (p.Y < map.Count - 1)
            {
                var nP = map[p.Y + 1][p.X];
                if (nP != null && nP.Owner == 1 && !discoveredPoints.Contains(nP))
                {
                    discoveredPoints.Add(nP);
                    queue.Enqueue(nP);
                }
            }

            if (p.X > 0)
            {
                var nP = map[p.Y][p.X - 1];
                if (nP != null && nP.Owner == 1 && !discoveredPoints.Contains(nP))
                {
                    discoveredPoints.Add(nP);
                    queue.Enqueue(nP);
                }
            }

            if (p.X < map[p.Y].Count - 1)
            {
                var nP = map[p.Y][p.X + 1];
                if (nP != null && nP.Owner == 1 && !discoveredPoints.Contains(nP))
                {
                    discoveredPoints.Add(nP);
                    queue.Enqueue(nP);
                }
            }
        }


        map[step.Y][step.X] = mapPoint;
        var killCount = oppUnitsCount - newOppUnitsCount;
        if (IsTowerCell(mapPoint.X, mapPoint.Y, map, 1))
            killCount++; //kill opp tower
        return killCount;
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

    static bool CanMove(Unit unit, Point point, IList<IList<Point>> map)
    {
        //opp tower
        if (IsTowerInfluenceCell(point.X, point.Y, map, 1) && unit.Level < 3)
            return false;
        
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

    static IList<Point> GetMapNeighbours(IList<IList<Point>> map, Point point, bool includeDiagonal)
    {
        var neighbours = new List<Point>();
        if (point.Y > 0)
            neighbours.Add(map[point.Y - 1][point.X]);
        if (point.Y < map.Count - 1)
            neighbours.Add(map[point.Y + 1][point.X]);
        if (point.X > 0)
            neighbours.Add(map[point.Y][point.X - 1]);
        if (point.X < map[point.Y].Count - 1)
            neighbours.Add(map[point.Y][point.X + 1]);

        if (includeDiagonal)
        {
            if (point.Y > 0 && point.X > 0)
            {
                neighbours.Add(map[point.Y - 1][point.X - 1]);
            }
            if (point.Y < map.Count - 1 && point.X > 0)
            {
                neighbours.Add(map[point.Y + 1][point.X - 1]);
            }

            if (point.Y > 0 && point.X < map[point.Y].Count - 1)
            {
                neighbours.Add(map[point.Y - 1][point.X + 1]);
            }
            if (point.Y < map.Count - 1 && point.X < map[point.Y].Count - 1)
            {
                neighbours.Add(map[point.Y + 1][point.X + 1]);
            }
        }

        return neighbours;
    }

    public static bool IsTowerCell(int x, int y, IList<IList<Point>> map, int owner)
    {
        return map[y][x] != null && map[y][x].IsActive && map[y][x].Owner == owner && map[y][x] is Building building &&
               building.BuildingType == 2;
    }

    public static bool IsTowerInfluenceCell(int x, int y, IList<IList<Point>> map, int owner)
    {
        if (map[y][x] == null)
            return false;

        if (IsTowerCell(x, y, map, owner))
            return true;

        var neighbours = GetMapNeighbours(map, map[y][x], false);
        foreach (var n in neighbours.Where(n => n != null))
            if (IsTowerCell(n.X, n.Y, map, owner))
                return true;
        return false;
    }
   
}