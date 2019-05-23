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

        public abstract double GetCost(APoint goal, Unit unit, Point[,] map);

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

        public override double GetCost(APoint goal, Unit unit, Point[,] map)
        {
            var aStarGoal = goal as AStarPoint;
            if (unit.Level < 3 && Player.IsTowerInfluenceCell(aStarGoal.X, aStarGoal.Y, map, unit.Owner == 0 ? 1 : 0))
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
            Unit unit, Point[,] map )
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

                    var tentativeGScore = x.G + x.GetCost(y,unit, map);
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
        public static IList<APoint> GetPath(APoint start, APoint goal, IEnumerable<APoint> allPoints, Unit unit, Point[,] map)
        {
            var emc = GetExpansionMatrix(start, goal, allPoints, unit, map);
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
    public class Command
    {
        public IList<Unit> RecruitmentUnits { get; set; }
        public IList<Tuple<Unit, Point>> Moves { get; set; }
    }

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

        public override string ToString()
        {
            return $"{X} {Y}";
        }
    }

    public class Building : Point
    {
        public int BuildingType { get; set; }

        public Building(int x, int y, int owner, int buildingType, bool isActive) : base(x, y, owner, isActive)
        {
            BuildingType = buildingType;
        }
    }

    public class Unit : Point
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
                
                
                var someBuilding = new Building(x, y, owner, buildingType, lines[y][x] == 'O' || lines[y][x] == 'X');
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
            var command = GetCommand(myUnits,
                oppUnits,
                map,
                myBase,
                oppBase,
                gold,
                table,
                allPoints,
                opponentGold,
                opponentIncome);
            var textCommand = "";

            foreach (var move in command.Moves)
            {
                textCommand += $"MOVE {move.Item1.Id} {move.Item2.X} {move.Item2.Y};";
            }

            foreach (var recUnit in command.RecruitmentUnits)
            {
                textCommand += $"TRAIN {recUnit.Level} {recUnit.X} {recUnit.Y};";
                gold -= GetUnitCost(recUnit.Level);
            }


            var mines = myBuildings.Where(b => b.BuildingType == 1).ToList();
            while (gold >= GetMineCost(mines.Count))
            {
                var bmp = GetBestMinePosition(map, mineSpots, myBuildings.Single(b => b.BuildingType == 0));
                if (bmp != null)
                {
                    textCommand += $"BUILD MINE {bmp.X} {bmp.Y};";
                    mines.Add(new Building(bmp.X, bmp.Y, 0, 1, true));
                    map[bmp.Y,bmp.X] = new Building(bmp.X, bmp.Y, 0, 1, true);
                    gold -= GetMineCost(mines.Count);
                }
                else
                    break;
            }
        

            Console.WriteLine(textCommand != "" ? textCommand : "WAIT");
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

    static Point GetBestMinePosition(Point[,] map, IList<Point> mineSpots,Building myBase)
    {
        Point bestMineSpot = null;
        int minDist = int.MaxValue;
        foreach (var ms in mineSpots)
        {
            var isOkPoint = map[ms.Y,ms.X] != null && !(map[ms.Y,ms.X] is Building) && !(map[ms.Y,ms.X] is Unit) &&
                             map[ms.Y,ms.X].Owner == myBase.Owner && map[ms.Y,ms.X].IsActive;
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


    static IList<Point> GetRecruitmentPoints(Point[,] map, int owner)
    {
        var recruitmentPoints = new List<Point>();
        for (var i = 0; i <Size; ++i)
        {
            for (var j = 0; j < Size; ++j)
            {
                var point = map[i,j];
                if (point == null) continue;
                if (point.Owner == owner)//my point
                {
                    continue;
                }

                var neighbours = GetMapNeighbours(map, point, false);
                if (!neighbours.Any(n => n != null && n.Owner == owner && n.IsActive))
                    continue;

                recruitmentPoints.Add(point);
            }
        }

        return recruitmentPoints;
    }

    static IList<Point> GetRecruitmentPoints(Point[,] map, Point point, int owner)
    {
        var neighbours = GetMapNeighbours(map, point, false).Where(n => n != null && n.Owner != owner).ToList();
        return neighbours;
    }

    static int GetUnitCost(int level)
    {
        return level == 3 ? RecruitmentCost3 : level == 2 ? RecruitmentCost2 : RecruitmentCost1;
    }

    static int GetUnitUpkeep(int level)
    {
        return level == 3 ? 20 : level == 2 ? 4 : 1;
    }

    static int GetOppMaxKillCount(IList<Unit> oppUnits, Point[,] map, Building myBase, Building oppBase, int oppGold)
    {
        int maxKillCount = GetBestRecruitmentUnits(map, myBase, oppGold, null, out _).Count;

        foreach (var unit in oppUnits)
        {
            var neighbours = GetMapNeighbours(map, unit, false);
            foreach (var n in neighbours)
            {
                if (n == null) continue;
                if (n.Owner == 1) continue;
                if (!CanMove(unit, n, map)) continue;

                var savedUnit = unit;
                var savedPoint = n;

                map[unit.Y,unit.X] = new Point(unit.X, unit.Y, 0, true);
                map[n.Y,n.X] = new Unit(n.X, n.Y, unit.Owner, unit.Id, unit.Level);

                var killedPoints = GetBestRecruitmentUnits(map, myBase, oppGold, n, out _);
                
                map[unit.Y,unit.X] = savedUnit;
                map[n.Y,n.X] = savedPoint;

                if (killedPoints.Count > maxKillCount)
                {
                    maxKillCount = killedPoints.Count;
                }
            }
        }

        return maxKillCount;
    }

    static Command GetCommand(
        IList<Unit> myUnits, IList<Unit> oppUnits, Point[,] map, Building myBase, Building oppBase, int gold, AStarPoint[,] table, IList<AStarPoint> allPoints,
        int oppGold, int oppIncome)
    {
        var moves = new List<Tuple<Unit, Point>>();

        //ходим всеми
        var endPoint = table[oppBase.Y, oppBase.X];
        myUnits = myUnits.OrderBy(u => GetManhDist(u, oppBase)).ToList();
        var noWayUnits = new List<Unit>();
        foreach (var myUnit in myUnits)
        {
            var startPoint = table[myUnit.Y, myUnit.X];
            var path = AStar.Calculator.GetPath(startPoint, endPoint, allPoints, myUnit, map);
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

            var step = path[1] as AStarPoint;
            if (CanMove(myUnit, map[step.Y,step.X], map))
            {
                moves.Add(new Tuple<Unit, Point>(myUnit, map[step.Y,step.X]));

                map[myUnit.Y,myUnit.X] = new Point(myUnit.X, myUnit.Y, 0, true);
                map[step.Y,step.X] = new Unit(step.X, step.Y, myUnit.Owner, myUnit.Id, myUnit.Level);

                table[step.Y, step.X].Owner = 0;
                table[myUnit.Y, myUnit.X].IsMySolder = false;
                table[step.Y, step.X].IsMySolder = true;
            }

        }

        //если солдат не может дойти до чужой базы, он идет захватывать свободные точки
        var canCapturePoints = new List<Point>();
        for (var i = 0; i < Size; ++i)
        {
            for (var j = 0; j < Size; ++j)
            {
                if (map[i,j] != null && map[i,j].Owner != 0)
                    canCapturePoints.Add(map[i,j]);
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
                bestUnit,
                map);

            var bestStep = bestPath[1] as AStarPoint;
            if (CanMove(bestUnit, map[bestStep.Y,bestStep.X], map))
            {
                moves.Add(new Tuple<Unit, Point>(bestUnit, map[bestStep.Y,bestStep.X]));

                map[bestUnit.Y,bestUnit.X] = new Point(bestUnit.X, bestUnit.Y, 0, true);
                map[bestStep.Y,bestStep.X] =
                    new Unit(bestStep.X, bestStep.Y, bestUnit.Owner, bestUnit.Id, bestUnit.Level);

                table[bestStep.Y, bestStep.X].Owner = 0;
                table[bestUnit.Y, bestUnit.X].IsMySolder = false;
                table[bestStep.Y, bestStep.X].IsMySolder = true;
            }

            canCapturePoints.Remove(bestCcp);
            noWayUnits.Remove(bestUnit);

        }

        var activatedPoints = UpdateAfterMoveMap(map, myBase);

        var allMoveKilledPoints = GetBestRecruitmentUnits(map, oppBase, gold, null, out var allMoveRecUnits);

        var allMoveSavedRecPoints = new List<Point>();
        foreach (var ru in allMoveRecUnits)
        {
            allMoveSavedRecPoints.Add(map[ru.Y,ru.X]);
            map[ru.Y,ru.X] = ru;
        }

        var allMoveAliveOppUnits = new List<Unit>();
        foreach (var oppUnit in oppUnits)
        {
            var isDead = false;
            foreach (var kp in allMoveKilledPoints)
            {
                if (kp is Unit killedOppUnit)
                {
                    if (killedOppUnit.Id == oppUnit.Id)
                    {
                        isDead = true;
                        break;
                    }
                }
            }
            if (!isDead)
                allMoveAliveOppUnits.Add(oppUnit);
        }

        var oppKilledCount = GetOppMaxKillCount(allMoveAliveOppUnits, map, myBase, oppBase, oppGold + oppIncome);
        
        foreach (var srp in allMoveSavedRecPoints)
        {
            map[srp.Y,srp.X] = srp;
        }

        var moveKilledCount =  moves.Count(m => m.Item2 is Unit || m.Item2 is Building);
        int maxDeltaKillCount = allMoveKilledPoints.Count + moveKilledCount - oppKilledCount;
        Console.Error.WriteLine($"ALL MOVE: {allMoveKilledPoints.Count + moveKilledCount} - {oppKilledCount} = {maxDeltaKillCount}");

        Tuple<Unit, Point> bestMove = null;
        List<Unit> bestRecUnits = allMoveRecUnits;

        foreach (var ap in activatedPoints)
            ap.IsActive = false;
        foreach (var move in moves)
        {
            var unit = move.Item1;
            var step = move.Item2;

            map[unit.Y,unit.X] = unit;
            map[step.Y,step.X] = step;

            table[step.Y, step.X].Owner = step.Owner;
            table[step.Y, step.X].IsMySolder = false;
            table[unit.Y, unit.X].IsMySolder = true;
            if (step is Unit || step is Building) moveKilledCount--;


            var neighbours = GetMapNeighbours(map, unit, false);
            foreach (var n in neighbours)
            {
                if (n == null) continue;
                if (n.Owner == 0) continue;
                if (n.X == step.X && n.Y == step.Y) continue;//этот ход уже рассмотрели
                if (!CanMove(unit, n, map)) continue;

                var savedUnit = unit;
                var savedPoint = n;

                map[unit.Y,unit.X] = new Point(unit.X, unit.Y, 0, true);
                map[n.Y,n.X] = new Unit(n.X, n.Y, unit.Owner, unit.Id, unit.Level);

                var killedPoints = GetBestRecruitmentUnits(map, oppBase, gold, n, out var recUnits);

                var savedRecPoints = new List<Point>();
                foreach (var ru in recUnits)
                {
                    savedRecPoints.Add(map[ru.Y,ru.X]);
                    map[ru.Y,ru.X] = ru;
                }

                var aliveOppUnits = new List<Unit>();
                foreach (var oppUnit in oppUnits)
                {
                    var isDead = false;
                    foreach (var kp in killedPoints)
                    {
                        if (kp is Unit killedOppUnit)
                        {
                            if (killedOppUnit.Id == oppUnit.Id)
                            {
                                isDead = true;
                                break;
                            }
                        }
                    }
                    if (!isDead)
                        aliveOppUnits.Add(oppUnit);
                }
                var oppKillCount = GetOppMaxKillCount(aliveOppUnits, map, myBase, oppBase, oppGold + oppIncome);
               
                map[unit.Y,unit.X] = savedUnit;
                map[n.Y,n.X] = savedPoint;
                foreach (var srp in savedRecPoints)
                {
                    map[srp.Y,srp.X] = srp;
                }

                if (moveKilledCount + killedPoints.Count - oppKillCount > maxDeltaKillCount)
                {
                    maxDeltaKillCount = moveKilledCount + killedPoints.Count - oppKillCount;
                    bestMove = new Tuple<Unit, Point>(unit, n);
                    bestRecUnits = recUnits;
                }
            }


            map[unit.Y,unit.X] = new Point(unit.X, unit.Y, 0, true);
            map[step.Y,step.X] = new Unit(step.X, step.Y, unit.Owner, unit.Id, unit.Level);

            table[step.Y, step.X].Owner = 0;
            table[unit.Y, unit.X].IsMySolder = false;
            table[step.Y, step.X].IsMySolder = true;
            if (step is Unit || step is Building) moveKilledCount++;
        }

        if (bestMove != null)
        {
            Console.Error.WriteLine($"BEST: {maxDeltaKillCount}");

            var bestMoveUnit = bestMove.Item1;
            var bestMovePoint = bestMove.Item2;

            moves.RemoveAll(m => m.Item1.Id == bestMoveUnit.Id);
            moves.Add(bestMove);

            map[bestMoveUnit.Y,bestMoveUnit.X] = new Point(bestMoveUnit.X, bestMoveUnit.Y, 0, true);
            map[bestMovePoint.Y,bestMovePoint.X] = new Unit(bestMovePoint.X, bestMovePoint.Y, bestMoveUnit.Owner, bestMoveUnit.Id, bestMoveUnit.Level);
            UpdateAfterMoveMap(map, myBase);
            
            foreach (var ru in bestRecUnits)
            {
                map[ru.Y,ru.X] = ru;
            }
        }



        

        

       

        //Unit recMoveUnit = null;
        //if (bestMove != null)
        //{
        //    if (!isSameOppKill)
        //    {
        //        moves.Add(bestMove);
        //        recMoveUnit = bestMove.Item1;

        //        recMoveUnit.X = bestMove.Item2.X;
        //        recMoveUnit.Y = bestMove.Item2.Y;

        //        map[recMoveUnit.Y][recMoveUnit.X] = new Point(recMoveUnit.Y, recMoveUnit.X, 0, true);
        //        map[bestMove.Item2.Y][bestMove.Item2.X] = new Unit(recMoveUnit.X,
        //            recMoveUnit.Y,
        //            recMoveUnit.Owner,
        //            recMoveUnit.Id,
        //            recMoveUnit.Level);

        //        table[bestMove.Item2.Y, bestMove.Item2.X].Owner = 0;
        //        table[recMoveUnit.Y, recMoveUnit.X].IsMySolder = false;
        //        table[bestMove.Item2.Y, bestMove.Item2.X].IsMySolder = true;

        //        foreach (var recUnit in bestRecUnits)
        //        {
        //            map[recUnit.Y][recUnit.X] = recUnit;

        //            table[recUnit.Y, recUnit.X].Owner = 0;
        //            table[recUnit.Y, recUnit.X].IsMySolder = true;
        //        }
        //    }
        //}


       

        //if (isSameOppKill || bestMove == null)
        //{
        //    //запускаем найм еще раз на обновленной карте
        //    GetBestRecruitmentUnits(map, oppBase, gold, null, out bestRecUnits);
        //    foreach (var recUnit in bestRecUnits)
        //    {
        //        map[recUnit.Y][recUnit.X] = recUnit;

        //        table[recUnit.Y, recUnit.X].Owner = 0;
        //        table[recUnit.Y, recUnit.X].IsMySolder = true;
        //    }
        //}

        return new Command() {RecruitmentUnits = bestRecUnits, Moves = moves};
    }

    static IList<Point> GetBestRecruitmentUnits(Point[,] map, Building oppBase, int gold, Point pointFrom, out List<Unit> resUnits)
    {
        var killedPoints = new List<Point>();
        if(pointFrom != null && pointFrom.Owner == oppBase.Owner &&
                        (pointFrom is Unit || pointFrom is Building building && building.BuildingType == 2))
            killedPoints.Add(pointFrom);

        if (gold < RecruitmentCost1)
        {
            resUnits = new List<Unit>();
            killedPoints.AddRange(GetKillUnits(map, oppBase));
            return killedPoints;
        }

        int owner = oppBase.Owner == 1 ? 0 : 1;

        var recruitmentPoints = pointFrom == null
            ? GetRecruitmentPoints(map, owner)
            : GetRecruitmentPoints(map, pointFrom, owner);
        if (!recruitmentPoints.Any())
        {
            resUnits = new List<Unit>();
            killedPoints.AddRange(GetKillUnits(map, oppBase));
            return killedPoints;
        }

        IList<Point> maxKilledPoints = new List<Point>();
        int minOppBaseDist = int.MaxValue;
        var bestResUnits = new List<Unit>();

        var hasRecPoint = false;
        foreach (var rp in recruitmentPoints)
        {
            int level = GetMinRecruitmentUnitLevel(rp, map, 1);
            var cost = GetUnitCost(level);
            if (cost > gold) continue;

            hasRecPoint = true;
            var changePoint = rp;
            var unit = new Unit(rp.X, rp.Y, owner, -1, level);
            map[rp.Y,rp.X] = unit ;


            var killedPointsCur = GetBestRecruitmentUnits(map, oppBase, gold - cost, rp, out var resUnitsCur);

            map[rp.Y,rp.X] = changePoint;

            if (killedPointsCur.Count > maxKilledPoints.Count || killedPointsCur.Count == maxKilledPoints.Count && GetManhDist(rp, oppBase) < minOppBaseDist)
            {
                maxKilledPoints = killedPointsCur;
                minOppBaseDist = GetManhDist(rp, oppBase);
                resUnitsCur.Insert(0, unit);
                bestResUnits = resUnitsCur;
            }
        }

        if (!hasRecPoint)
        {
            resUnits = new List<Unit>();
            killedPoints.AddRange(GetKillUnits(map, oppBase));
            return killedPoints;
        }

        resUnits = bestResUnits;
        killedPoints.AddRange(maxKilledPoints);
        return killedPoints;
    }

    static int GetMinRecruitmentUnitLevel(Point point, Point[,] map, int oppId)
    {
        if (point.Owner == -1) return 1;
        if (IsTowerInfluenceCell(point.X, point.Y, map, oppId))
            return 3;

        if (point is Building building && building.BuildingType == 2) return 3;
        if (point is Unit unit)
        {
            switch (unit.Level)
            {
                case 2:
                case 3:
                    return 3;
                default:
                    return 2;
            }
        }

        return 1;
    }


    static IList<Unit> GetKillUnits(Point[,] map, Building oppBase)
    {
        var aliveUnits = new Dictionary<Unit, bool>();
        for (var i = 0; i < Size; ++i)
        {
            for (var j = 0; j < Size; ++j)
            {
                if (map[i,j] is Unit mapUnit && mapUnit.Owner == oppBase.Owner)
                    aliveUnits.Add(mapUnit, true);
            }
        }

        //BFS


        var queue = new Stack<Point>();
        queue.Push(oppBase);
        var discoveredPoints = new bool[Size, Size];
        discoveredPoints[oppBase.Y, oppBase.X] = true;

        while (queue.Count > 0)
        {
            var p = queue.Pop();
            if (p is Unit pUnit && pUnit.Owner == oppBase.Owner)
                aliveUnits.Remove(pUnit);
           
            if (p.Y > 0)
            {
                var nP = map[p.Y - 1,p.X];
                if (nP != null && nP.Owner == oppBase.Owner && !discoveredPoints[nP.Y, nP.X])
                {
                    discoveredPoints[nP.Y, nP.X] = true;
                    queue.Push(nP);
                }
            }
            if (p.Y < Size - 1)
            {
                var nP = map[p.Y + 1,p.X];
                if (nP != null && nP.Owner == oppBase.Owner && !discoveredPoints[nP.Y, nP.X])
                {
                    discoveredPoints[nP.Y, nP.X] = true;
                    queue.Push(nP);
                }
            }

            if (p.X > 0)
            {
                var nP = map[p.Y,p.X - 1];
                if (nP != null && nP.Owner == oppBase.Owner && !discoveredPoints[nP.Y, nP.X])
                {
                    discoveredPoints[nP.Y, nP.X] = true;
                    queue.Push(nP);
                }
            }

            if (p.X < Size - 1)
            {
                var nP = map[p.Y,p.X + 1];
                if (nP != null && nP.Owner == oppBase.Owner && !discoveredPoints[nP.Y, nP.X])
                {
                    discoveredPoints[nP.Y, nP.X] = true;
                    queue.Push(nP);
                }
            }
        }


        return aliveUnits.Keys.ToList();
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

    static bool CanMove(Unit unit, Point point, Point[,] map)
    {
        if (point == null)
            return false;
        //opp tower
        if (IsTowerInfluenceCell(point.X, point.Y, map, 1) && unit.Level < 3)
            return false;
        
        if (point is Building pointBuilding)
        {
            if (pointBuilding.Owner == unit.Owner)
                return false;
            
            return true;
        }
        if (point is Unit pointUnit)
        {
            if (pointUnit.Owner == unit.Owner)
                return false;

            return unit.Level == 3 || unit.Level > pointUnit.Level;
        }
        //нейтральная точка
        return true;
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

    static Point[,] GetMap(IList<string> lines, IList<Building> myBuildings, IList<Unit> myUnits,
        IList<Building> oppBuildings, IList<Unit> oppUnits)
    {
        var map = new Point[Size,Size];
        for (var i = 0; i < lines.Count; ++i)
        {
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

                map[i, j] = point;

            }
        }

        foreach (var b in myBuildings)
        {
            map[b.Y,b.X] = b;
        }

        foreach (var u in myUnits)
        {
            map[u.Y,u.X] = u;
        }

        foreach (var b in oppBuildings)
        {
            map[b.Y,b.X] = b;
        }

        foreach (var u in oppUnits)
        {
            map[u.Y,u.X] = u;
        }

        return map;
    }

    static IList<Point> UpdateAfterMoveMap(Point[,] map, Building myBase)
    {
        //BFS
        var queue = new Queue<Point>();
        queue.Enqueue(myBase);
        var discoveredPoints = new HashSet<Point>(){myBase};

        var activatedPoints = new List<Point>();
        while (queue.Count > 0)
        {
            var p = queue.Dequeue();
            if (p.Owner == myBase.Owner && !p.IsActive)
            {
                p.IsActive = true;
                activatedPoints.Add(p);
                //Console.Error.WriteLine($"POINT {p.X} {p.Y} IS ACTIVATED");
            }

            if (p.Y > 0)
            {
                var nP = map[p.Y - 1,p.X];
                if (nP != null && nP.Owner == myBase.Owner && !discoveredPoints.Contains(nP))
                {
                    discoveredPoints.Add(nP);
                    queue.Enqueue(nP);
                }
            }
            if (p.Y < Size - 1)
            {
                var nP = map[p.Y + 1,p.X];
                if (nP != null && nP.Owner == myBase.Owner && !discoveredPoints.Contains(nP))
                {
                    discoveredPoints.Add(nP);
                    queue.Enqueue(nP);
                }
            }

            if (p.X > 0)
            {
                var nP = map[p.Y,p.X - 1];
                if (nP != null && nP.Owner == myBase.Owner && !discoveredPoints.Contains(nP))
                {
                    discoveredPoints.Add(nP);
                    queue.Enqueue(nP);
                }
            }

            if (p.X < Size - 1)
            {
                var nP = map[p.Y,p.X + 1];
                if (nP != null && nP.Owner == myBase.Owner && !discoveredPoints.Contains(nP))
                {
                    discoveredPoints.Add(nP);
                    queue.Enqueue(nP);
                }
            }
        }

        return activatedPoints;
    }
    

    static int GetMineCost(int mines)
    {
        return 20 + 4 * mines;
    }

    static IList<Point> GetMapNeighbours(Point[,] map, Point point, bool includeDiagonal)
    {
        var neighbours = new List<Point>();
        if (point.Y > 0)
            neighbours.Add(map[point.Y - 1,point.X]);
        if (point.Y < Size - 1)
            neighbours.Add(map[point.Y + 1,point.X]);
        if (point.X > 0)
            neighbours.Add(map[point.Y,point.X - 1]);
        if (point.X < Size - 1)
            neighbours.Add(map[point.Y,point.X + 1]);

        if (includeDiagonal)
        {
            if (point.Y > 0 && point.X > 0)
            {
                neighbours.Add(map[point.Y - 1,point.X - 1]);
            }
            if (point.Y < Size - 1 && point.X > 0)
            {
                neighbours.Add(map[point.Y + 1,point.X - 1]);
            }

            if (point.Y > 0 && point.X < Size - 1)
            {
                neighbours.Add(map[point.Y - 1,point.X + 1]);
            }
            if (point.Y < Size - 1 && point.X < Size - 1)
            {
                neighbours.Add(map[point.Y + 1,point.X + 1]);
            }
        }

        return neighbours;
    }

    public static bool IsTowerCell(int x, int y, Point[,] map, int owner)
    {
        return map[y,x] != null && map[y,x].IsActive && map[y,x].Owner == owner && map[y,x] is Building building &&
               building.BuildingType == 2;
    }

    public static bool IsTowerInfluenceCell(int x, int y, Point[,] map, int owner)
    {
        if (map[y,x] == null)
            return false;

        if (IsTowerCell(x, y, map, owner))
            return true;

        var neighbours = GetMapNeighbours(map, map[y,x], false);
        foreach (var n in neighbours.Where(n => n != null))
            if (IsTowerCell(n.X, n.Y, map, owner))
                return true;
        return false;
    }
   
}