using System;
using System.Linq;
using System.Collections.Generic;
using AStar;
using MapPoints;



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

        public abstract double GetCost(APoint goal, Unit unit, Point[,] map, bool isCostPath, bool considerOpp);

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

        public override double GetCost(APoint goal, Unit unit, Point[,] map, bool isCostPath, bool considerOpp)
        {
            var aStarGoal = goal as AStarPoint;

            if (isCostPath)
            {
                var goalP = map[aStarGoal.Y, aStarGoal.X];
                if (goalP == null)
                    return BIG_WEIGHT;
                if (goalP.Owner == unit.Owner)
                {
                    if (goalP is Unit)
                        return BIG_WEIGHT;
                    if (goalP is Building)
                        return BIG_WEIGHT;
                    return 10;
                }

                if (goalP.Owner == -1)
                    return 10;
                //opponent
                if (goalP is Building goalBuilding)
                {
                    if (goalBuilding.BuildingType == 0 || goalBuilding.BuildingType == 1)
                        return 10;
                    return 30;
                }
                else if (goalP is Unit goalUNit)
                {
                    switch (goalUNit.Level)
                    {
                        case 1:
                            return 20;
                        default:
                            return 30;
                    }
                }
                else//empty
                {
                    var isInfluenceCell = false;
                    if (goalP.Y > 0 && map[goalP.Y - 1, goalP.X] != null &&
                        map[goalP.Y - 1, goalP.X].Owner != unit.Owner &&
                        map[goalP.Y - 1, goalP.X].IsActive && map[goalP.Y - 1, goalP.X] is Building mBuilding1 &&
                        mBuilding1.BuildingType == 2 && (X != goalP.X || Y != goalP.Y - 1))
                    {
                        isInfluenceCell = true;
                    }

                    if (goalP.Y < 12 - 1 && 
                        map[goalP.Y + 1, goalP.X] != null &&
                        map[goalP.Y + 1, goalP.X].Owner != unit.Owner &&
                        map[goalP.Y + 1, goalP.X].IsActive && 
                        map[goalP.Y + 1, goalP.X] is Building mBuilding2 &&
                        mBuilding2.BuildingType == 2 && (X != goalP.X || Y != goalP.Y + 1))
                    {
                        isInfluenceCell = true;
                    }

                    if (goalP.X > 0 && map[goalP.Y, goalP.X - 1] != null &&
                        map[goalP.Y, goalP.X-1].Owner != unit.Owner &&
                        map[goalP.Y, goalP.X-1].IsActive && 
                        map[goalP.Y, goalP.X-1] is Building mBuilding3 &&
                        mBuilding3.BuildingType == 2 && (X != goalP.X-1 || Y != goalP.Y))
                    {
                        isInfluenceCell = true;
                    }

                    if (goalP.X < 12 - 1 && map[goalP.Y, goalP.X + 1] != null &&
                        map[goalP.Y, goalP.X + 1].Owner != unit.Owner &&
                        map[goalP.Y, goalP.X + 1].IsActive &&
                        map[goalP.Y, goalP.X + 1] is Building mBuilding4 &&
                        mBuilding4.BuildingType == 2 && (X != goalP.X + 1 || Y != goalP.Y))
                    {
                        isInfluenceCell = true;
                    }


                    return isInfluenceCell ? 30 : 10;
                }
                        
            }

           

           
            if (considerOpp && unit.Level < 3 && Player.IsTowerInfluenceCell(aStarGoal.X, aStarGoal.Y, map, unit.Owner == 0 ? 1 : 0))
                return BIG_WEIGHT;

            if (map[aStarGoal.Y, aStarGoal.X] != null && map[aStarGoal.Y, aStarGoal.X].Owner == unit.Owner &&
                map[aStarGoal.Y, aStarGoal.X] is Building)
                return BIG_WEIGHT;

            if (map[aStarGoal.Y, aStarGoal.X] is Unit mUnit)
            {
                if (mUnit.Owner == unit.Owner)
                    return BIG_WEIGHT;

                if (considerOpp && unit.Level != 3 && unit.Level <= mUnit.Level)
                    return BIG_WEIGHT;
            }


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

    public class ExpansionMatrixConteiner
    {
        public IDictionary<APoint, double> ExpansionMatrix { get; set; }
        public APoint RealGoalPoint { get; set; }
    }

    public static class Calculator
    {
        private static ExpansionMatrixConteiner GetExpansionMatrix(APoint start, APoint goal, IEnumerable<APoint> allPoints,
            Unit unit, Point[,] map , bool isCostPath, bool considerOpp)
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

                    var tentativeGScore = x.G + x.GetCost(y,unit, map, isCostPath, considerOpp);
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

       
        public static IList<APoint> GetPath(APoint start, APoint goal, IEnumerable<APoint> allPoints, Unit unit,
            Point[,] map, bool isCostPath, bool considerOpp)
        {
            start.G = 0;
            start.H = 0;
            start.CameFromPoint = null;
            var emc = GetExpansionMatrix(start, goal, allPoints, unit, map, isCostPath, considerOpp);
            return ReconstructPath(emc.RealGoalPoint);
        }
      
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
        
    }
}


namespace MapPoints
{
    public class LossContainer
    {
        public bool IsWin { get; set; }
        public List<Unit> KilledUnits { get; set; }
        public List<Building> KilledBuildings { get; set; }
        public List<Building> DeactivatedBuildings { get; set; }
        public List<Point> LostPoint { get; set; }

        public LossContainer()
        {
            IsWin = false;
            KilledUnits = new List<Unit>();
            KilledBuildings = new List<Building>();
            DeactivatedBuildings = new List<Building>();
            LostPoint = new List<Point>();
        }

        public void Add(LossContainer lc)
        {
            IsWin = IsWin || lc.IsWin;
            KilledUnits.AddRange(lc.KilledUnits);
            KilledBuildings.AddRange(lc.KilledBuildings);
            DeactivatedBuildings.AddRange(lc.DeactivatedBuildings);
            LostPoint.AddRange(lc.LostPoint);
        }
    }

    public class Command
    {
        public IList<Unit> RecruitmentUnits { get; set; }
        public IList<Building> BuilingTowers { get; set; }
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
    enum Action
    {
        StayNoTower,
        StayBuildTower,
        MoveNoTower,
        MoveBuildTower
    }
#if DEBUG
    private const int TIME = 500000;
#else
    private const int TIME = 50;
#endif

    private const int OppBorderTowersDist = 3;
    private const int MaxDeepLevel = 5;
    const int BIG_WEIGHT = 10000;

    const int RecruitmentCost1 = 10;
    const int RecruitmentCost2 = 20;
    const int RecruitmentCost3 = 30;
    const int TowerCost = 15;
    private const int Size = 12;
    private static Random _rnd = new Random();
    private static int[,] _oppBaseMap; 
    private static int[,] _oppBorderMap; 
    private static bool[][,] _oppNeutralMap; 
    

    static void Main(string[] args)
    {
        

        string input;
        string[] inputs;

        var mineSpots = new List<Point>();

        input = Console.ReadLine(); 
        int numberMineSpots = int.Parse(input);
        for (int i = 0; i < numberMineSpots; i++)
        {
            input = Console.ReadLine();
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


            int gold = int.Parse(Console.ReadLine());

            Console.Error.WriteLine(numberMineSpots);
            foreach (var ms in mineSpots)
                Console.Error.WriteLine($"{ms.X} {ms.Y}");

            Console.Error.WriteLine(gold);
            int income = int.Parse(Console.ReadLine()); Console.Error.WriteLine(income);
            int opponentGold = int.Parse(Console.ReadLine()); Console.Error.WriteLine(opponentGold);
            int opponentIncome = int.Parse(Console.ReadLine()); Console.Error.WriteLine(opponentIncome);

            var table = new AStarPoint[Size, Size];
            var allPoints = new List<AStarPoint>();

            var lines = new List<string>();
            var oCount = 0;
            for (int i = 0; i < Size; i++)
            {
                string line = Console.ReadLine(); Console.Error.WriteLine(line);

                for (var j = 0; j < line.Length; ++j)
                {
                    var owner = -1;
                    if (line[j] == 'o' || line[j] == 'O')
                    {
                        owner = 0;
                        oCount++;
                    }
                    else if (line[j] == 'x' || line[j] == 'X') owner = 1;
                    var isBase = i == 0 && j == 0 || i == 11 && j == 11 || i == 10 && j == 11;
                   
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

#if DEBUG
            _oppBaseMap = GetOppBaseMap(map, oppBase);
#endif

            if (isFire && oCount == 1 || !isFire && oCount == 2)
            {
                _oppBaseMap = GetOppBaseMap(map, oppBase);
                if (isFire)
                {
                    Console.WriteLine($"TRAIN 1 1 0; TRAIN 1 0 1;");
                }
                else
                {
                    Console.WriteLine($"TRAIN 1 10 11; TRAIN 1 11 9;");
                }

                continue;
            }

            _oppNeutralMap = GetOppNeutralMap(map, myBase, oppBase);
           
            
            var command = GetCommand(myUnits,
                oppUnits,
                map,
                myBase,
                oppBase,
                gold,
                income,
                table,
                allPoints,
                mineSpots,
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

            foreach (var tower in command.BuilingTowers)
            {
                textCommand += $"BUILD TOWER {tower.X} {tower.Y};";
                gold -= TowerCost;
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


    static List<Point> GetRecruitmentPoints(Point[,] map, int owner)
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

                if (point.Owner == -1 && !_oppNeutralMap[owner][point.Y, point.X]) continue;

                var neighbours = GetMapNeighbours(map, point, false);
                if (!neighbours.Any(n => n != null && n.Owner == owner && n.IsActive))
                    continue;

                recruitmentPoints.Add(point);
            }
        }

        return recruitmentPoints;
    }

    static List<Point> GetRecruitmentPoints(Point[,] map, Point point, int owner)
    {
        var neighbours = new List<Point>();
        if (point.X > 0 && map[point.Y, point.X - 1] != null && map[point.Y, point.X - 1].Owner != owner && 
            (map[point.Y, point.X - 1].Owner != -1 || _oppNeutralMap[owner][point.Y, point.X-1]))
            neighbours.Add(map[point.Y, point.X - 1]);
        if (point.X < Size - 1 && map[point.Y, point.X + 1] != null && map[point.Y, point.X + 1].Owner != owner &&
            (map[point.Y, point.X + 1].Owner != -1 || _oppNeutralMap[owner][point.Y, point.X + 1]))
            neighbours.Add(map[point.Y, point.X + 1]);
        if (point.Y > 0 && map[point.Y - 1, point.X] != null && map[point.Y - 1, point.X].Owner != owner &&
            (map[point.Y - 1, point.X].Owner != -1 || _oppNeutralMap[owner][point.Y-1, point.X]))
            neighbours.Add(map[point.Y - 1, point.X]);
        if (point.Y < Size - 1 && map[point.Y + 1, point.X] != null && map[point.Y + 1, point.X].Owner != owner &&
            (map[point.Y + 1, point.X].Owner != -1 || _oppNeutralMap[owner][point.Y+1, point.X]))
            neighbours.Add(map[point.Y + 1, point.X]);
        

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
        int maxKillCount = GetBestRecruitmentUnitsCount(map, oppBase, myBase, oppGold, null, 0, false, out var oppRecUnits0);

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

                map[unit.Y,unit.X] = new Point(unit.X, unit.Y, 1, true);
                map[n.Y,n.X] = new Unit(n.X, n.Y, unit.Owner, unit.Id, unit.Level);

                var killedPointsCount = GetBestRecruitmentUnitsCount(map, oppBase, myBase, oppGold, n, 0, n.Owner == 0, out var oppRecUnits);
                
                map[unit.Y,unit.X] = savedUnit;
                map[n.Y,n.X] = savedPoint;

                if (killedPointsCount > maxKillCount)
                {
                    maxKillCount = killedPointsCount;
                }
            }
        }

        return maxKillCount;
    }

    static IList<Unit> GetAliveOppUnit(Point[,] map)
    {
        //находим живых врагов
        var allMoveAliveOppUnits = new List<Unit>();
        for (var i = 0; i < Size; i++)
        {
            for (var j = 0; j < Size; ++j)
            {
                if (map[i, j] != null && map[i, j] is Unit mapUnit && mapUnit.Owner == 1)
                    allMoveAliveOppUnits.Add(mapUnit);
            }
        }

        return allMoveAliveOppUnits;
    }

    static Command GetCommand(
        IList<Unit> myUnits, IList<Unit> oppUnits, Point[,] map, Building myBase, Building oppBase, int gold, int income, AStarPoint[,] table, IList<AStarPoint> allPoints,
        IList<Point> mineSpots,
        int oppGold, int oppIncome)
    {
        var watch = System.Diagnostics.Stopwatch.StartNew();

        Building saveBuilding = null;

        //if (gold > TowerCost)
        //{
        //    var endCostPoint = table[myBase.Y, myBase.X];
        //    IList<APoint> minCostPath = null;
        //    var minSumCost = double.MaxValue;
        //    foreach (var oppUnit in oppUnits)
        //    {
        //        var startCostPoint = table[oppUnit.Y, oppUnit.X];
        //        var costPath = Calculator.GetPath(
        //            startCostPoint, endCostPoint, allPoints, oppUnit, map, true, true);
        //        var sumCost = costPath[costPath.Count - 1].G;
        //        if (costPath[1].G <= 10 + 1E-3)
        //            sumCost -= 10;

        //        if (sumCost < minSumCost)
        //        {
        //            minSumCost = sumCost;
        //            minCostPath = costPath;
        //        }
        //    }

        //    if (minSumCost <= oppGold + oppIncome)
        //    {
        //        var maxDefCount = -1;
        //        var minDefWeight = int.MaxValue;
        //        Point resPoint = null;
        //        for (var i = 1; i < minCostPath.Count; ++i)
        //        {
        //            AStarPoint asPoint = minCostPath[i] as AStarPoint;
        //            var mapPoint = map[asPoint.Y, asPoint.X];
        //            if (mapPoint == null || mapPoint.Owner != 0 || !mapPoint.IsActive || mapPoint is Unit ||
        //                mapPoint is Building || mineSpots.Any(ms => ms.X == asPoint.X && ms.Y == asPoint.Y))
        //                continue;

        //            var prevAsPoint = minCostPath[i - 1] as AStarPoint;
        //            var mapPrevAsPoint = map[prevAsPoint.Y, prevAsPoint.X];
        //            var count = 0;
        //            var weight = 0;
        //            if (mapPrevAsPoint != null && mapPrevAsPoint.Owner == 0)
        //            {
        //                count++;
        //                if (mapPrevAsPoint is Unit mpasUnit)
        //                {
        //                    weight += GetUnitCost(mpasUnit.Level);
        //                }
        //                else if (mapPrevAsPoint is Building mpasBuilding)
        //                {
        //                    if (mpasBuilding.BuildingType == 2)
        //                        weight += 30;
        //                }
        //                else
        //                {
        //                    weight += 0;
        //                }
        //            }
        //            else
        //            {
        //                weight += BIG_WEIGHT;
        //            }

        //            if (count > maxDefCount || count == maxDefCount && weight < minDefWeight)
        //            {
        //                maxDefCount = count;
        //                minDefWeight = weight;
        //                resPoint = mapPoint;
        //            }
                    
        //        }

        //        if (minSumCost <= oppGold + oppIncome - maxDefCount * 30)
        //            resPoint = null;

        //        if (resPoint == null)
        //        {
        //            var ns = new Dictionary<Point, int>();
        //            foreach (AStarPoint asp in minCostPath)
        //            {
        //                if (map[asp.Y, asp.X].Owner != 0) continue;

        //                var neighbours = GetMapNeighbours(map, map[asp.Y, asp.X], false);
        //                foreach (var n in neighbours)
        //                {
        //                    if (n == null || n.Owner != 0 || !n.IsActive || n is Unit || n is Building ||
        //                        mineSpots.Any(ms => ms.X == n.X && ms.Y == n.Y))
        //                        continue;
        //                    if (!ns.ContainsKey(n))
        //                        ns.Add(n, 0);
        //                    ns[n]++;
        //                }
        //            }

        //            int maxNeighbours = 0;
        //            foreach (var item in ns)
        //                if (item.Value > maxNeighbours)
        //                {
        //                    maxNeighbours = item.Value;
        //                    resPoint = item.Key;
        //                }

        //            if (minSumCost <= oppGold + oppIncome - maxNeighbours * 30)
        //                resPoint = null;
        //        }

                

        //        if (resPoint != null)
        //        {
        //            Console.Error.WriteLine("SAVE TOWER");
        //            saveBuilding = new Building(resPoint.X, resPoint.Y, 0, 2, true);
        //            gold -= TowerCost;
        //            map[resPoint.Y, resPoint.X] = saveBuilding;
        //        }
        //    }
        //}


        _oppBorderMap = GetOppBorderMap(map);

        //стоим всеми
        //var action = Action.StayNoTower;
        //var moveKilledCount = 0;
        //var allMoveResOppGold = oppGold + oppIncome;

        //IList<Point> activatedPoints = new List<Point>();//активируем мои точки в результате движения юнитов
        //var allMoveLc = GetBestRecruitmentUnits(map, myBase, oppBase, gold, income, null, out var allMoveRecUnits);
        //if (allMoveLc.IsWin)
        //{
        //    Console.Error.WriteLine("WIN");
        //    return new Command()
        //    {
        //        BuilingTowers = new List<Building>(), Moves = new List<Tuple<Unit, Point>>(),
        //        RecruitmentUnits = allMoveRecUnits
        //    };
        //}

        //foreach (var ru in allMoveRecUnits)//снимаем 1 за захваченные тренировкой точки врага
        //{
        //    var mapP = map[ru.Y, ru.X];
        //    if (mapP.Owner == 1 && !(mapP is Unit) && (!(mapP is Building b) || b.BuildingType != 2) && mapP.IsActive)
        //        allMoveResOppGold--;
        //}

        //var allMoveCapturedPoints = UpdateMap(map, allMoveRecUnits, allMoveLc);


        //foreach (var unit in allMoveLc.KilledUnits)
        //{
        //    allMoveResOppGold += GetUnitUpkeep(unit.Level);
        //    allMoveResOppGold--;//за контрольную точку
        //}

        //foreach (var b in allMoveLc.DeactivatedBuildings)
        //    allMoveResOppGold--;//за контрольную точку

        //foreach (var point in allMoveLc.LostPoint)
        //    allMoveResOppGold--;


        //var oppKilledCount = GetOppMaxKillCount(GetAliveOppUnit(map), map, myBase, oppBase, allMoveResOppGold);
        //var noTowersOppKilledCount = oppKilledCount;
        //UpdateMapBack(map, allMoveLc, activatedPoints, allMoveCapturedPoints);

        //var maxSumKill = allMoveLc.KilledUnits.Count + allMoveLc.KilledBuildings.Count + moveKilledCount;
        //int maxDeltaKillCount = maxSumKill - oppKilledCount;
        //Console.Error.WriteLine(
        //    $"ALL STAY: {maxSumKill} - {oppKilledCount} = {maxDeltaKillCount}. Gold: {allMoveResOppGold}");

        //if (allMoveLc.DeactivatedBuildings.Any())
        //{
        //    foreach (var db in allMoveLc.DeactivatedBuildings)
        //    {
        //        Console.Error.WriteLine($"DB: {db.X} {db.Y}");
        //    }
        //}

        //List<Unit> bestRecUnits = allMoveRecUnits;
        //List<Building> bestBuildTowers = new List<Building>();
        //var bestMovies = new List<Tuple<Unit, Point>>();
        //int maxProtectPointsCount = 0;

        //watch.Stop();
        //var elapsedMs = watch.ElapsedMilliseconds;
        //if (elapsedMs > TIME)
        //{
        //    Console.Error.WriteLine("time is over");
        //    if (saveBuilding != null) bestBuildTowers.Add(saveBuilding);
        //    return new Command()
        //    {
        //        BuilingTowers = bestBuildTowers, Moves = bestMovies,
        //        RecruitmentUnits = allMoveRecUnits
        //    };
        //}
        //watch.Start();

        ////вариант с башнями
        //var isWatchStopped = false;
        //if (gold >= TowerCost)
        //{
        //    for (var i = Size - 1; i >= 0; --i)
        //    {
        //        if (isWatchStopped)
        //            break;
        //        for (var j = Size - 1; j >= 0; --j)
        //        {
        //            watch.Stop();
        //            elapsedMs = watch.ElapsedMilliseconds;
        //            if (elapsedMs > TIME)
        //            {
        //                isWatchStopped = true;
        //                break;
        //            }
        //            else
        //                watch.Start();

        //            var point = map[i, j];
        //            if (point == null || point.Owner != 0 || !point.IsActive || point is Unit || point is Building || mineSpots.Any(ms => ms.X == point.X && ms.Y == point.Y))
        //                continue;

        //            if (_oppBorderMap[i, j] > OppBorderTowersDist) continue;

        //            //if (!IsCloseBorderPoint(point, map)) continue;

        //            var neighbours = GetMapNeighbours(map, point, false);
        //            var protectPointsCount = 0;
        //            foreach (var n in neighbours)
        //            {
        //                if (n == null || n.Owner != 0) continue;
        //                if (IsTowerInfluenceCell(n.X, n.Y, map, 0)) continue;
        //                protectPointsCount++;
        //            }

        //            if (protectPointsCount <= 1) continue;

        //            if (!IsTowerInfluenceCell(point.X, point.Y, map, 0))
        //                protectPointsCount++;

        //            var tower = new Building(j, i, 0, 2, true);
        //            map[i, j] = tower;

        //            var towerOppKilledCount = GetOppMaxKillCount(GetAliveOppUnit(map),
        //                map,
        //                myBase,
        //                oppBase,
        //                oppGold + oppIncome);
        //            if (towerOppKilledCount >= noTowersOppKilledCount)
        //            {
        //                map[i, j] = point;
        //                continue; //не строим башню, если она никого не спасет
        //            }

        //            if (moveKilledCount - towerOppKilledCount > maxDeltaKillCount ||
        //                moveKilledCount - towerOppKilledCount == maxDeltaKillCount && protectPointsCount > maxProtectPointsCount)
        //            {
        //                action = Action.StayBuildTower;
        //                maxSumKill = moveKilledCount;
        //                maxDeltaKillCount = moveKilledCount - towerOppKilledCount;
        //                maxProtectPointsCount = protectPointsCount;
        //                bestRecUnits = new List<Unit>();
        //                bestBuildTowers = new List<Building>() { tower };
        //                Console.Error.WriteLine($"STAY TOWER: {moveKilledCount} - {towerOppKilledCount} = {maxDeltaKillCount}");
        //            }

        //            map[i, j] = point;
        //        }
        //    }
        //}
        

        //watch.Stop();
        //elapsedMs = watch.ElapsedMilliseconds;
        //if (elapsedMs > TIME)
        //{
        //    Console.Error.WriteLine("time is over");
        //    if (saveBuilding != null) bestBuildTowers.Add(saveBuilding);
        //    return new Command()
        //    {
        //        BuilingTowers = bestBuildTowers,
        //        Moves = bestMovies,
        //        RecruitmentUnits = allMoveRecUnits
        //    };
        //}
        //watch.Start();

        ////если построили башню - нанимаем солдат на остаток
        //if (bestBuildTowers.Any() && gold - TowerCost >= RecruitmentCost1)
        //{
        //    var savedPoints = new List<Point>();
        //    foreach (var bbt in bestBuildTowers)
        //    {
        //        savedPoints.Add(map[bbt.Y, bbt.X]);
        //        map[bbt.Y, bbt.X] = bbt;
        //    }

        //    activatedPoints = UpdateAfterMoveMap(map, myBase);//активируем мои точки в результате движения юнитов
        //    allMoveLc = GetBestRecruitmentUnits(map, myBase, oppBase, gold - TowerCost, income, null, out allMoveRecUnits);

        //    if (allMoveLc.IsWin)
        //    {
        //        Console.Error.WriteLine("WIN");
        //        return new Command()
        //        {
        //            BuilingTowers = bestBuildTowers,
        //            Moves = bestMovies,
        //            RecruitmentUnits = allMoveRecUnits
        //        };
        //    }

        //    allMoveResOppGold = oppGold + oppIncome;
        //    foreach (var ru in allMoveRecUnits)//снимаем 1 за захваченные тренировкой точки врага
        //    {
        //        var mapP = map[ru.Y, ru.X];
        //        if (mapP.Owner == 1 && !(mapP is Unit) && (!(mapP is Building b) || b.BuildingType != 2) && mapP.IsActive)
        //            allMoveResOppGold--;
        //    }

        //    allMoveCapturedPoints = UpdateMap(map, allMoveRecUnits, allMoveLc);

        //    foreach (var unit in allMoveLc.KilledUnits)
        //    {
        //        allMoveResOppGold += GetUnitUpkeep(unit.Level);
        //        allMoveResOppGold--;//за контрольную точку
        //    }

        //    foreach (var b in allMoveLc.DeactivatedBuildings)
        //        allMoveResOppGold--;//за контрольную точку

        //    foreach (var point in allMoveLc.LostPoint)
        //        allMoveResOppGold--;


        //    oppKilledCount = GetOppMaxKillCount(GetAliveOppUnit(map), map, myBase, oppBase, allMoveResOppGold);
        //    UpdateMapBack(map, allMoveLc, activatedPoints, allMoveCapturedPoints);

        //    maxSumKill = allMoveLc.KilledUnits.Count + allMoveLc.KilledBuildings.Count + moveKilledCount;
        //    maxDeltaKillCount = maxSumKill - oppKilledCount;
        //    bestRecUnits.AddRange(allMoveRecUnits);

        //    foreach (var sp in savedPoints)
        //        map[sp.Y, sp.X] = sp;
        //}

        //watch.Stop();
        //elapsedMs = watch.ElapsedMilliseconds;
        //if (elapsedMs > TIME)
        //{
        //    Console.Error.WriteLine("time is over");
        //    if (saveBuilding != null) bestBuildTowers.Add(saveBuilding);
        //    return new Command()
        //    {
        //        BuilingTowers = bestBuildTowers,
        //        Moves = bestMovies,
        //        RecruitmentUnits = allMoveRecUnits
        //    };
        //}
        //watch.Start();

        var moves = new List<Tuple<Unit, Point>>();

        //ходим всеми
       
        var endPoint = table[oppBase.Y, oppBase.X];
        myUnits = myUnits.OrderBy(u => GetManhDist(u, oppBase)).ToList();
        var noWayUnits = new List<Unit>();

        foreach (var myUnit in myUnits)
        {
            
            var startPoint = table[myUnit.Y, myUnit.X];
           
            var neighbours = GetMapNeighbours(map, myUnit, false);
            int minOppBorderDist = int.MaxValue;
            Point minOppBorderDistPoint = null;
            foreach (var n in neighbours)
            {
                if (n == null || n.Owner == myUnit.Owner || 
                    !CanMove(myUnit, n, map))
                    continue;

                if (_oppBaseMap[n.Y,n.X] > _oppBaseMap[myUnit.Y, myUnit.X])
                    continue;

               

                if (_oppBorderMap[n.Y, n.X] < minOppBorderDist)
                {
                    minOppBorderDist = _oppBorderMap[n.Y, n.X];
                    minOppBorderDistPoint = n;
                }
            }

            if (minOppBorderDistPoint != null)
            {
                moves.Add(new Tuple<Unit, Point>(myUnit, map[minOppBorderDistPoint.Y, minOppBorderDistPoint.X]));

                map[myUnit.Y, myUnit.X] = new Point(myUnit.X, myUnit.Y, 0, true);
                map[minOppBorderDistPoint.Y, minOppBorderDistPoint.X] = new Unit(minOppBorderDistPoint.X, minOppBorderDistPoint.Y, myUnit.Owner, myUnit.Id, myUnit.Level);

                table[minOppBorderDistPoint.Y, minOppBorderDistPoint.X].Owner = 0;
                table[myUnit.Y, myUnit.X].IsMySolder = false;
                table[minOppBorderDistPoint.Y, minOppBorderDistPoint.X].IsMySolder = true;
                continue;
            }

            var path = AStar.Calculator.GetPath(startPoint, endPoint, allPoints, myUnit, map, false, true);
            if (path.Count < 2) continue;
            var step = path[1] as AStarPoint;

            if (!IsTowerInfluenceCell(myUnit.X, myUnit.Y, map, 0))
            {
                var noOppPath = Calculator.GetPath(startPoint, endPoint, allPoints, myUnit, map, false, false);
                var isBorderUnit = noOppPath.Count - 1 == _oppBaseMap[myUnit.Y, myUnit.X];
                if (isBorderUnit && _oppBaseMap[step.Y, step.X] > _oppBaseMap[myUnit.Y, myUnit.X])
                    continue;
            }

            var isNoWay = false;
            for (var i = 1; i < path.Count; ++i)
            {
                if (path[i].G >= BIG_WEIGHT)
                {
                    isNoWay = true;
                    break;
                }
            }

            if (isNoWay)
            {
                noWayUnits.Add(myUnit);
                continue;
            }

           
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
                    if (!CanMove(unit, ccp, map)) continue;
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
                map,
                false,
                true);

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

        
        var moveKilledCount = 0;
        var allMoveOppAddGold = 0;

        foreach (var move in moves)
        {
            if (move.Item2 is Building)
            {
                moveKilledCount++;
                if (move.Item2.IsActive)
                    allMoveOppAddGold--;//за контрольную точку
            }
            else if (move.Item2 is Unit mUnit)
            {
                moveKilledCount++;
                allMoveOppAddGold += GetUnitUpkeep(mUnit.Level);
                allMoveOppAddGold--;//за контрольную точку
            }
            else if (move.Item2.Owner == 1 && move.Item2.IsActive)//opp point
            {
                allMoveOppAddGold--;//за контрольную точку
            }
        }

        var allMoveResOppGold = oppGold + oppIncome + allMoveOppAddGold;

        var activatedPoints = UpdateAfterMoveMap(map, myBase);//активируем мои точки в результате движения юнитов
        var allMoveLc = GetBestRecruitmentUnits(map, myBase, oppBase, gold, income, null, out var allMoveRecUnits);

        if (allMoveLc.IsWin)
        {
            Console.Error.WriteLine("WIN");
            return new Command()
            {
                BuilingTowers = new List<Building>(),
                Moves = moves,
                RecruitmentUnits = allMoveRecUnits
            };
        }

        foreach (var ru in allMoveRecUnits)//снимаем 1 за захваченные тренировкой точки врага
        {
            var mapP = map[ru.Y, ru.X];
            if (mapP.Owner == 1 && !(mapP is Unit) && (!(mapP is Building b) || b.BuildingType != 2) && mapP.IsActive)
                allMoveResOppGold--;
        }

        var allMoveCapturedPoints = UpdateMap(map, allMoveRecUnits, allMoveLc);

        
        foreach (var unit in allMoveLc.KilledUnits)
        {
            allMoveResOppGold += GetUnitUpkeep(unit.Level);
            allMoveResOppGold--;//за контрольную точку
        }

        foreach (var b in allMoveLc.DeactivatedBuildings)
            allMoveResOppGold--;//за контрольную точку

        foreach (var point in allMoveLc.LostPoint)
            allMoveResOppGold--;


        var oppKilledCount = GetOppMaxKillCount(GetAliveOppUnit(map), map, myBase, oppBase, allMoveResOppGold);
        var noTowersOppKilledCount = oppKilledCount;
        UpdateMapBack(map, allMoveLc, activatedPoints, allMoveCapturedPoints);

        var allMoveSumKill = allMoveLc.KilledUnits.Count + allMoveLc.KilledBuildings.Count + moveKilledCount;
        var allMoveDeltaKillCount = allMoveSumKill - oppKilledCount;

        //if (allMoveDeltaKillCount >= maxDeltaKillCount)
        //{
            var action = Action.MoveNoTower;
            var maxDeltaKillCount = allMoveDeltaKillCount;
            var maxSumKill = allMoveSumKill;

            Console.Error.WriteLine(
                $"ALL MOVE: {maxSumKill} - {oppKilledCount} = {maxDeltaKillCount}. Gold: {allMoveResOppGold}");

            var bestRecUnits = allMoveRecUnits;
            var bestBuildTowers = new List<Building>();
            var bestMovies = moves;
            var maxProtectPointsCount = 0;
        //}

        watch.Stop();
        var elapsedMs = watch.ElapsedMilliseconds;
        if (elapsedMs > TIME)
        {
            Console.Error.WriteLine("time is over");
            if (saveBuilding != null) bestBuildTowers.Add(saveBuilding);
            return new Command()
            {
                BuilingTowers = bestBuildTowers,
                Moves = bestMovies,
                RecruitmentUnits = allMoveRecUnits
            };
        }
        Console.Error.WriteLine($"TIME { elapsedMs}");
        watch.Start();

        //вариант с башнями
        var isWatchStopped = false;
        if (gold >= TowerCost)
        {
            for (var i = Size - 1; i >= 0; --i)
            {
                if (isWatchStopped)
                    break;
                for (var j = Size - 1; j >= 0; --j)
                {
                    watch.Stop();
                    elapsedMs = watch.ElapsedMilliseconds;
                    if (elapsedMs > TIME)
                    {
                        isWatchStopped = true;
                        break;
                    }
                    else
                        watch.Start();

                    var point = map[i, j];
                    if (point == null || point.Owner != 0 || !point.IsActive || point is Unit || point is Building || mineSpots.Any(ms => ms.X == point.X && ms.Y == point.Y))
                        continue;

                    if (_oppBorderMap[i,j] > OppBorderTowersDist) continue;

                    //if (!IsCloseBorderPoint(point, map)) continue;

                    var neighbours = GetMapNeighbours(map, point, false);
                    var protectPointsCount = 0;
                    foreach (var n in neighbours)
                    {
                        if (n == null || n.Owner != 0) continue;
                        if (IsTowerInfluenceCell(n.X, n.Y, map, 0)) continue;
                        protectPointsCount++;
                    }

                    if (protectPointsCount <= 1) continue;
                    
                    if (!IsTowerInfluenceCell(point.X, point.Y, map, 0))
                        protectPointsCount++;

                    var tower = new Building(j, i, 0, 2, true);
                    map[i, j] = tower;

                    var towerOppKilledCount = GetOppMaxKillCount(GetAliveOppUnit(map),
                        map,
                        myBase,
                        oppBase,
                        oppGold + oppIncome + allMoveOppAddGold);
                    if (towerOppKilledCount >= noTowersOppKilledCount)
                    {
                        map[i, j] = point;
                        continue; //не строим башню, если она никого не спасет
                    }

                    if ((action == Action.StayBuildTower || action == Action.StayNoTower) && moveKilledCount - towerOppKilledCount >= maxDeltaKillCount ||
                        moveKilledCount - towerOppKilledCount > maxDeltaKillCount ||
                        moveKilledCount - towerOppKilledCount == maxDeltaKillCount && protectPointsCount > maxProtectPointsCount)
                    {
                        action = Action.MoveBuildTower;
                        maxSumKill = moveKilledCount;
                        maxDeltaKillCount = moveKilledCount - towerOppKilledCount;
                        maxProtectPointsCount = protectPointsCount;
                        bestRecUnits = new List<Unit>();
                        bestBuildTowers = new List<Building>() {tower};
                        bestMovies = moves;
                        Console.Error.WriteLine($"ALL MOVE TOWER: {moveKilledCount} - {towerOppKilledCount} = {maxDeltaKillCount}");
                    }

                    map[i, j] = point;
                }
            }
        }

        watch.Stop();
        elapsedMs = watch.ElapsedMilliseconds;
        if (elapsedMs > TIME)
        {
            Console.Error.WriteLine("time is over");
            if (saveBuilding != null) bestBuildTowers.Add(saveBuilding);
            return new Command()
            {
                BuilingTowers = bestBuildTowers,
                Moves = bestMovies,
                RecruitmentUnits = allMoveRecUnits
            };
        }
        Console.Error.WriteLine($"TIME { elapsedMs}");
        watch.Start();

        //если построили башню - нанимаем солдат на остаток
        if (action == Action.MoveBuildTower)
        {
            foreach (var bbt in bestBuildTowers)
            {
                map[bbt.Y, bbt.X] = bbt;
            }

            gold -= TowerCost;
            if (gold > RecruitmentCost1)
            {
                activatedPoints = UpdateAfterMoveMap(map, myBase); //активируем мои точки в результате движения юнитов
                allMoveLc = GetBestRecruitmentUnits(map, myBase, oppBase, gold, income, null, out allMoveRecUnits);

                if (allMoveLc.IsWin)
                {
                    Console.Error.WriteLine("WIN");
                    return new Command()
                    {
                        BuilingTowers = bestBuildTowers,
                        Moves = moves,
                        RecruitmentUnits = allMoveRecUnits
                    };
                }

                foreach (var ru in allMoveRecUnits) //снимаем 1 за захваченные тренировкой точки врага
                {
                    var mapP = map[ru.Y, ru.X];
                    if (mapP.Owner == 1 && !(mapP is Unit) && (!(mapP is Building b) || b.BuildingType != 2) &&
                        mapP.IsActive)
                        allMoveResOppGold--;
                }

                allMoveCapturedPoints = UpdateMap(map, allMoveRecUnits, allMoveLc);

                foreach (var unit in allMoveLc.KilledUnits)
                {
                    allMoveResOppGold += GetUnitUpkeep(unit.Level);
                    allMoveResOppGold--; //за контрольную точку
                }

                foreach (var b in allMoveLc.DeactivatedBuildings)
                    allMoveResOppGold--; //за контрольную точку

                foreach (var point in allMoveLc.LostPoint)
                    allMoveResOppGold--;


                oppKilledCount = GetOppMaxKillCount(GetAliveOppUnit(map), map, myBase, oppBase, allMoveResOppGold);
                UpdateMapBack(map, allMoveLc, activatedPoints, allMoveCapturedPoints);

                maxSumKill = allMoveLc.KilledUnits.Count + allMoveLc.KilledBuildings.Count + moveKilledCount;
                maxDeltaKillCount = maxSumKill - oppKilledCount;
                bestRecUnits.AddRange(allMoveRecUnits);
            }
        }

        watch.Stop();
        elapsedMs = watch.ElapsedMilliseconds;
        if (elapsedMs > TIME)
        {
            Console.Error.WriteLine("time is over");
            if (saveBuilding != null) bestBuildTowers.Add(saveBuilding);
            return new Command()
            {
                BuilingTowers = bestBuildTowers,
                Moves = bestMovies,
                RecruitmentUnits = allMoveRecUnits
            };
        }
        watch.Start();


        Tuple<Unit, Point> bestMove = null;
        int bestOppGold = -1;
        var isWin = false;
      
        foreach (var move in moves)
        {
            watch.Stop();
            elapsedMs = watch.ElapsedMilliseconds;
            if(elapsedMs > TIME) 
                break;
            Console.Error.WriteLine($"time is {elapsedMs}");
            watch.Start();

            if (map[move.Item1.Y, move.Item1.X] is Building)//здесь уже строим башню
            {
                continue;
            }

            var unit = move.Item1;
            var step = move.Item2;

            map[unit.Y,unit.X] = unit;
            map[step.Y,step.X] = step;

            table[step.Y, step.X].Owner = step.Owner;
            table[step.Y, step.X].IsMySolder = false;
            table[unit.Y, unit.X].IsMySolder = true;
            if (step is Unit || step is Building) moveKilledCount--;

            if (step is Building)
            {
                if (step.IsActive)
                    allMoveOppAddGold++;
            }
            else if (step is Unit stepUnit)
            {
                allMoveOppAddGold -= GetUnitUpkeep(stepUnit.Level);
                allMoveOppAddGold++;
            }
            else if (step.Owner == 1 && step.IsActive)
            {
                allMoveOppAddGold++;
            }


            var neighbours = GetMapNeighbours(map, unit, false);
            neighbours.Add(unit);
            foreach (var n in neighbours)
            {
                var savedUnit = unit;
                var savedPoint = n;

                var connectedUnits = new List<Unit>();

                if (!Equals(n, unit))
                {
                    if (n == null) continue;
                    if (n.Owner == 0) continue;
                    if (n.X == step.X && n.Y == step.Y) continue; //этот ход уже рассмотрели
                    if (!CanMove(unit, n, map)) continue;

                    if (!moves.Any(m => m.Item2.X == unit.X && m.Item2.Y == unit.Y))
                        map[unit.Y, unit.X] = new Point(unit.X, unit.Y, 0, true);
                    map[n.Y, n.X] = new Unit(n.X, n.Y, unit.Owner, unit.Id, unit.Level);
                }
                else
                {
                    //отменяем все ходы, которые завязаны на точку юнита
                    var hasConnectedMovies = true;
                    var considerPoint = unit;
                    while (hasConnectedMovies)
                    {
                        hasConnectedMovies = false;
                        foreach (var m in moves)
                        {
                            if (m.Item2.X == considerPoint.X && m.Item2.Y == considerPoint.Y)
                            {
                                hasConnectedMovies = true;
                                considerPoint = m.Item1;

                                var connectedUnit = m.Item1;
                                connectedUnits.Add(connectedUnit);

                                //отмена хода
                                map[connectedUnit.Y, connectedUnit.X] = connectedUnit;
                                table[connectedUnit.Y, connectedUnit.X].IsMySolder = true;
                            }
                        }
                    }
                }

                var resOppGold = oppGold + oppIncome + allMoveOppAddGold;


                //TODO: активировать точки тут смысла нет, т.к. тренировка всегда будет рядом с мувом
                var lc = GetBestRecruitmentUnits(map, myBase, oppBase, gold, income, null, out var recUnits);
                
                foreach (var ru in recUnits)//снимаем 1 за захваченные тренировкой точки врага
                {
                    var mapP = map[ru.Y, ru.X];
                    if (mapP.Owner == 1 && !(mapP is Unit) && (!(mapP is Building b) || b.BuildingType != 2) && mapP.IsActive)
                        resOppGold--;
                }

                var capturedPoints = UpdateMap(map, recUnits, lc);

                foreach (var kUnit in lc.KilledUnits)
                {
                    resOppGold += GetUnitUpkeep(kUnit.Level);
                    resOppGold--;//за контрольную точку
                }

                foreach (var b in lc.DeactivatedBuildings)
                    resOppGold--;//за контрольную точку

                foreach (var point in lc.LostPoint)
                    resOppGold--;

                var oppKillCount = GetOppMaxKillCount(GetAliveOppUnit(map), map, myBase, oppBase, resOppGold);

                
                map[unit.Y, unit.X] = savedUnit;
                map[n.Y, n.X] = savedPoint;

                foreach (var connectedUnit in connectedUnits)
                {
                    //возврат
                    map[unit.Y, unit.X] = connectedUnit;
                    table[connectedUnit.Y, connectedUnit.X].IsMySolder = false;
                }
                

                UpdateMapBack(map, lc, new List<Point>(), capturedPoints);

                var sumKill = moveKilledCount + lc.KilledUnits.Count + lc.KilledBuildings.Count;

                if (lc.IsWin ||
                    (action == Action.StayBuildTower || action == Action.StayNoTower) && sumKill - oppKillCount >= maxDeltaKillCount ||
                    sumKill - oppKillCount > maxDeltaKillCount || sumKill - oppKillCount == maxDeltaKillCount && sumKill > maxSumKill)
                {
                    maxDeltaKillCount = sumKill - oppKillCount;
                    maxSumKill = sumKill;
                    bestMove = new Tuple<Unit, Point>(unit, n);
                    bestRecUnits = recUnits;
                    bestOppGold = resOppGold;
                    if (action == Action.StayBuildTower)
                        bestBuildTowers = new List<Building>();
                    if (action == Action.StayBuildTower || action == Action.StayNoTower)
                        action = Action.MoveNoTower;

                    if (lc.IsWin)
                    {
                        isWin = true;
                        break;
                    }
                }
            }

            if (!moves.Any(m => m.Item2.X == unit.X && m.Item2.Y == unit.Y))
                map[unit.Y,unit.X] = new Point(unit.X, unit.Y, 0, true);
            map[step.Y,step.X] = new Unit(step.X, step.Y, unit.Owner, unit.Id, unit.Level);

            table[step.Y, step.X].Owner = 0;
            table[unit.Y, unit.X].IsMySolder = false;
            table[step.Y, step.X].IsMySolder = true;
            if (step is Unit || step is Building) moveKilledCount++;

            if (step is Building)
            {
                if (step.IsActive)
                    allMoveOppAddGold--;
            }
            else if (step is Unit stepUnit0)
            {
                allMoveOppAddGold += GetUnitUpkeep(stepUnit0.Level);
                allMoveOppAddGold--;
            }
            else if (step.Owner == 1 && step.IsActive)
            {
                allMoveOppAddGold--;
            }
            if (isWin)
                break;
        }

        if (bestMove != null)
        {
            Console.Error.WriteLine($"BEST: {maxDeltaKillCount}. Gold: {bestOppGold}");

            var bestMoveUnit = bestMove.Item1;
            var bestMovePoint = bestMove.Item2;

            if (bestMoveUnit.X == bestMovePoint.X && bestMoveUnit.Y == bestMovePoint.Y)
            {
                moves.RemoveAll(m =>
                    m.Item2.X == bestMovePoint.X && m.Item2.Y == bestMovePoint.Y &&
                    (m.Item1.X != bestMovePoint.X || m.Item1.Y != bestMovePoint.Y));
            }

            moves.RemoveAll(m => m.Item1.Id == bestMoveUnit.Id);
            int moveIndex = 0;//находм момент, когда ячейка хода освободится
            for (var i = 0; i < moves.Count; ++i)
            {
                if (moves[i].Item1.X == bestMovePoint.X && moves[i].Item1.Y == bestMovePoint.Y)
                {
                    moveIndex = i + 1;
                }
            }
            moves.Insert(moveIndex, bestMove);
            bestMovies = moves;

            map[bestMoveUnit.Y,bestMoveUnit.X] = new Point(bestMoveUnit.X, bestMoveUnit.Y, 0, true);
            map[bestMovePoint.Y,bestMovePoint.X] = new Unit(bestMovePoint.X, bestMovePoint.Y, bestMoveUnit.Owner, bestMoveUnit.Id, bestMoveUnit.Level);
            
        }
        else if (action == Action.StayNoTower)
        {
            //moves = new List<Tuple<Unit, Point>>();
        }
        else if (action == Action.StayBuildTower)
        {
            //moves = new List<Tuple<Unit, Point>>();
            foreach (var bt in bestBuildTowers)
                map[bt.Y, bt.X] = bt;
        }

        foreach (var ru in bestRecUnits)
        {
            map[ru.Y, ru.X] = ru;
        }

        
        

        if (saveBuilding != null)
            bestBuildTowers.Insert(0, saveBuilding);
        return new Command() {RecruitmentUnits = bestRecUnits, Moves = bestMovies, BuilingTowers = bestBuildTowers};
    }

    static bool IsCloseBorderPoint(Point point, Point[,] map)
    {
        var neighbours0 = GetMapNeighbours(map, point, false);
        if (neighbours0.Any(n => n != null && n.Owner == 1))
            return true;

        var allNeighbours1 = new List<Point>();
        foreach (var n0 in neighbours0)
        {
            if (n0 == null) continue;
            var neighbours1 = GetMapNeighbours(map, n0, false);
            if (neighbours1.Any(n => n != null && n.Owner == 1))
                return true;
            allNeighbours1.AddRange(neighbours1);
        }

        foreach (var n1 in allNeighbours1)
        {
            if (n1 == null) continue;
            var neighbours2 = GetMapNeighbours(map, n1, false);
            if (neighbours2.Any(n => n != null && n.Owner == 1))
                return true;
        }

        return false;
    }

    static int GetBestRecruitmentUnitsCount(Point[,] map, Building myBase, Building oppBase, int gold, Point pointFrom, int deepLevel, bool gotOppPoints, out List<Unit> resUnits)
    {
        if (pointFrom != null && pointFrom.X == oppBase.X && pointFrom.Y == oppBase.Y)
        {
            resUnits = new List<Unit>();
            return BIG_WEIGHT;
        }

        var killedPoints = 0;
        //if (pointFrom != null && pointFrom.Owner == oppBase.Owner &&
        //    (pointFrom is Unit || pointFrom is Building building && building.BuildingType == 2))
        //    killedPoints++;

        if (gold < RecruitmentCost1 || deepLevel > MaxDeepLevel)
        {
            resUnits = new List<Unit>();
            if (gotOppPoints)
                killedPoints += GetKillCount(map, oppBase);
            return killedPoints;
        }

        int owner = oppBase.Owner == 1 ? 0 : 1;

        List<Point> recruitmentPoints;
        IList<Point> activatedPoints = null;
        if (pointFrom == null)
        {
            recruitmentPoints = GetRecruitmentPoints(map, owner);
        }
        else
        {
            recruitmentPoints = GetRecruitmentPoints(map, pointFrom, owner);

            var needUpdate = pointFrom.Y > 0 && map[pointFrom.Y - 1, pointFrom.X] != null &&
                             !map[pointFrom.Y - 1, pointFrom.X].IsActive &&
                             map[pointFrom.Y - 1, pointFrom.X].Owner == myBase.Owner ||
                             pointFrom.Y < Size - 1 && map[pointFrom.Y + 1, pointFrom.X] != null &&
                             !map[pointFrom.Y + 1, pointFrom.X].IsActive &&
                             map[pointFrom.Y + 1, pointFrom.X].Owner == myBase.Owner ||
                             pointFrom.X > 0 && map[pointFrom.Y, pointFrom.X - 1] != null &&
                             !map[pointFrom.Y, pointFrom.X - 1].IsActive &&
                             map[pointFrom.Y, pointFrom.X - 1].Owner == myBase.Owner ||
                             pointFrom.X < Size - 1 && map[pointFrom.Y, pointFrom.X + 1] != null &&
                             !map[pointFrom.Y, pointFrom.X + 1].IsActive &&
                             map[pointFrom.Y, pointFrom.X + 1].Owner == myBase.Owner;
                

            if (needUpdate)
            {
                activatedPoints = UpdateAfterMoveMap(map, myBase);
                foreach (var ap in activatedPoints)
                {
                    var rps = GetRecruitmentPoints(map, ap, owner);
                    foreach (var rp in rps)
                    {
                        if (!recruitmentPoints.Contains(rp))
                            recruitmentPoints.Add(rp);
                    }
                }
            }
        }
        
        
        var maxKilledPoints = 0;
        int minOppBaseDist = int.MaxValue;
        var bestResUnits = new List<Unit>();

        var hasRecPoint = false;
        foreach (var rp in recruitmentPoints)
        {
            int level = GetMinRecruitmentUnitLevel(rp, map, 0);
            var cost = GetUnitCost(level);
            if (cost > gold) continue;

            hasRecPoint = true;
            var changePoint = rp;
            var unit = new Unit(rp.X, rp.Y, owner, -1, level);
            map[rp.Y, rp.X] = unit;

            var killedPointsCur = GetBestRecruitmentUnitsCount(map,
                myBase,
                oppBase,
                gold - cost,
                rp,
                deepLevel + 1,
                gotOppPoints || rp.Owner == oppBase.Owner,
                out var resUnitsCur);

            map[rp.Y, rp.X] = changePoint;

            if (killedPointsCur > maxKilledPoints || killedPointsCur == maxKilledPoints && GetManhDist(rp, oppBase) < minOppBaseDist)
            {
                maxKilledPoints = killedPointsCur;
                minOppBaseDist = GetManhDist(rp, oppBase);
                resUnitsCur.Insert(0, unit);
                bestResUnits = resUnitsCur;
            }
        }

        if (activatedPoints != null)
            foreach (var ap in activatedPoints)
                ap.IsActive = false;

        if (!hasRecPoint)
        {
            resUnits = new List<Unit>();
            if (gotOppPoints)
                killedPoints += GetKillCount(map, oppBase);
            return killedPoints;
        }

        resUnits = bestResUnits;
        killedPoints += maxKilledPoints;
        return killedPoints;
    }


    static LossContainer GetBestRecruitmentUnits(Point[,] map, Building myBase, Building oppBase, int gold, int income, Point pointFrom, out List<Unit> resUnits)
    {
        var lc = new LossContainer();
        if (pointFrom != null && pointFrom.X == oppBase.X && pointFrom.Y == oppBase.Y)
        {
            resUnits = new List<Unit>();
            lc.IsWin = true;
            return lc;
        }

        if (pointFrom != null && pointFrom.Owner == oppBase.Owner)
        {
            if (pointFrom is Unit pUnit)
                lc.KilledUnits.Add(pUnit);
            else if (pointFrom is Building pBuilding )
                lc.KilledBuildings.Add(pBuilding);
        }

        if (gold < RecruitmentCost1)
        {
            resUnits = new List<Unit>();
            lc.Add(GetLossContainer(map, oppBase));
            return lc;
        }

        int owner = oppBase.Owner == 1 ? 0 : 1;
        IList<Point> activatedPoints = null;
        List<Point> recruitmentPoints;

        if (pointFrom == null)
        {
            recruitmentPoints = GetRecruitmentPoints(map, owner);
        }
        else
        {
            recruitmentPoints = GetRecruitmentPoints(map, pointFrom, owner);

            var needUpdate = pointFrom.Y > 0 && map[pointFrom.Y - 1, pointFrom.X] != null &&
                             !map[pointFrom.Y - 1, pointFrom.X].IsActive &&
                             map[pointFrom.Y - 1, pointFrom.X].Owner == myBase.Owner ||
                             pointFrom.Y < Size - 1 && map[pointFrom.Y + 1, pointFrom.X] != null &&
                             !map[pointFrom.Y + 1, pointFrom.X].IsActive &&
                             map[pointFrom.Y + 1, pointFrom.X].Owner == myBase.Owner ||
                             pointFrom.X > 0 && map[pointFrom.Y, pointFrom.X - 1] != null &&
                             !map[pointFrom.Y, pointFrom.X - 1].IsActive &&
                             map[pointFrom.Y, pointFrom.X - 1].Owner == myBase.Owner ||
                             pointFrom.X < Size - 1 && map[pointFrom.Y, pointFrom.X + 1] != null &&
                             !map[pointFrom.Y, pointFrom.X + 1].IsActive &&
                             map[pointFrom.Y, pointFrom.X + 1].Owner == myBase.Owner;


            if (needUpdate)
            {
                activatedPoints = UpdateAfterMoveMap(map, myBase);
                foreach (var ap in activatedPoints)
                {
                    var rps = GetRecruitmentPoints(map, ap, owner);
                    foreach (var rp in rps)
                    {
                        if (!recruitmentPoints.Contains(rp))
                            recruitmentPoints.Add(rp);
                    }
                }
            }
        }

     

        LossContainer bestLc = null;
        int minOppBaseDist = int.MaxValue;
        int minOppBorderDist = int.MaxValue;
        var bestResUnits = new List<Unit>();

        var hasRecPoint = false;
        foreach (var rp in recruitmentPoints)
        {
            int level = GetMinRecruitmentUnitLevel(rp, map, 1);
            var cost = GetUnitCost(level);
            var upkeep = GetUnitUpkeep(level) - 1; //-1 за захваченную точку
            if (cost > gold) continue;
            if (upkeep > income) continue;


            hasRecPoint = true;
            var changePoint = rp;
            var unit = new Unit(rp.X, rp.Y, owner, -1, level);
            map[rp.Y,rp.X] = unit ;


            var lcCur = GetBestRecruitmentUnits(map, myBase, oppBase, gold - cost, income - upkeep, rp, out var resUnitsCur);

            map[rp.Y,rp.X] = changePoint;

            //TODO: нормальный критерий
            if (bestLc == null || lcCur.IsWin || lcCur.KilledUnits.Count > bestLc.KilledUnits.Count ||
                lcCur.KilledUnits.Count == bestLc.KilledUnits.Count && _oppBorderMap[rp.Y, rp.X] < minOppBorderDist ||
                lcCur.KilledUnits.Count == bestLc.KilledUnits.Count && _oppBorderMap[rp.Y, rp.X] == minOppBorderDist &&
                _oppBaseMap[rp.Y, rp.X] < minOppBaseDist)
            {
                bestLc = lcCur;
                minOppBorderDist = _oppBorderMap[rp.Y, rp.X];
                minOppBaseDist = _oppBaseMap[rp.Y, rp.X];
                resUnitsCur.Insert(0, unit);
                bestResUnits = resUnitsCur;
                if (bestLc.IsWin)
                    break;
            }
        }

        if (activatedPoints != null)
            foreach (var ap in activatedPoints)
                ap.IsActive = false;

        if (!hasRecPoint)
        {
            resUnits = new List<Unit>();
            lc.Add(GetLossContainer(map, oppBase));
            return lc;
        }

        resUnits = bestResUnits;
        lc.Add(bestLc);
        return lc;
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


    static int GetKillCount(Point[,] map, Building oppBase)
    {
        var killedCount = 0;
        for (var i = 0; i < Size; ++i)
        {
            for (var j = 0; j < Size; ++j)
            {
                if (map[i, j] is Unit mapUnit && mapUnit.Owner == oppBase.Owner)
                    killedCount++;
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
                killedCount--;

            if (p.Y > 0)
            {
                var nP = map[p.Y - 1, p.X];
                if (nP != null && nP.Owner == oppBase.Owner && !discoveredPoints[nP.Y, nP.X])
                {
                    discoveredPoints[nP.Y, nP.X] = true;
                    queue.Push(nP);
                }
            }
            if (p.Y < Size - 1)
            {
                var nP = map[p.Y + 1, p.X];
                if (nP != null && nP.Owner == oppBase.Owner && !discoveredPoints[nP.Y, nP.X])
                {
                    discoveredPoints[nP.Y, nP.X] = true;
                    queue.Push(nP);
                }
            }

            if (p.X > 0)
            {
                var nP = map[p.Y, p.X - 1];
                if (nP != null && nP.Owner == oppBase.Owner && !discoveredPoints[nP.Y, nP.X])
                {
                    discoveredPoints[nP.Y, nP.X] = true;
                    queue.Push(nP);
                }
            }

            if (p.X < Size - 1)
            {
                var nP = map[p.Y, p.X + 1];
                if (nP != null && nP.Owner == oppBase.Owner && !discoveredPoints[nP.Y, nP.X])
                {
                    discoveredPoints[nP.Y, nP.X] = true;
                    queue.Push(nP);
                }
            }
        }


        return killedCount;
    }


    static LossContainer GetLossContainer(Point[,] map, Building oppBase)
    {
       
        //BFS


        var queue = new Stack<Point>();
        queue.Push(oppBase);
        var discoveredPoints = new bool[Size, Size];
        discoveredPoints[oppBase.Y, oppBase.X] = true;

        while (queue.Count > 0)
        {
            var p = queue.Pop();
           
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

        var lc = new LossContainer();
        for (var i = 0; i < Size; ++i)
        {
            for (var j = 0; j < Size; ++j)
            {
                if (!discoveredPoints[i, j] && map[i,j] != null && map[i, j].Owner == oppBase.Owner && map[i, j].IsActive)
                {
                    if (map[i,j] is Unit unit)
                        lc.KilledUnits.Add(unit);
                    else if (map[i,j] is Building building)
                        lc.DeactivatedBuildings.Add(building);
                    else 
                        lc.LostPoint.Add(map[i,j]);
                }
            }
        }

        return lc;
    }

 

    static bool CanMove(Unit unit, Point point, Point[,] map)
    {
        if (point == null)
            return false;
        //opp tower
        if (IsTowerInfluenceCell(point.X, point.Y, map, unit.Owner == 0 ? 1 : 0) && unit.Level < 3)
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

    static IList<Point> UpdateMap(Point[,] map, IList<Unit> recUnits, LossContainer lc)
    {
        var capturedPoints = new List<Point>();
        foreach (var ru in recUnits)
        {
            if (!(map[ru.Y, ru.X] is Unit) && !(map[ru.Y, ru.X] is Building))
                capturedPoints.Add(map[ru.Y, ru.X]);
            map[ru.Y, ru.X] = ru;
        }

        foreach (var unit in lc.KilledUnits)
        {
            if (map[unit.Y, unit.X].Owner == unit.Owner)//точка не занята мной, лишь отрезана
                map[unit.Y, unit.X] = new Point(unit.X, unit.Y, unit.Owner, false);
        }

        foreach (var point in lc.LostPoint)
        {
            if (map[point.Y, point.X].Owner == point.Owner)//точка не занята мной, лишь отрезана
                map[point.Y, point.X] = new Point(point.X, point.Y, point.Owner, false);
        }

        foreach (var b in lc.DeactivatedBuildings)
            map[b.Y, b.X].IsActive = false;

        return capturedPoints;
    }

    static void UpdateMapBack(Point[,] map, LossContainer lc, IList<Point> activatedPoint, IList<Point> capturedPoints)
    {
        foreach (var unit in lc.KilledUnits)
        {
            map[unit.Y, unit.X] = unit;
        }

        foreach (var building in lc.KilledBuildings)
        {
            map[building.Y, building.X] = building;
        }

        foreach (var lp in lc.LostPoint)
        {
            map[lp.Y,lp.X] = lp;
        }

        foreach (var b in lc.DeactivatedBuildings)
            map[b.Y, b.X].IsActive = true;

        foreach (var ap in activatedPoint)
            ap.IsActive = false;

        foreach (var cp in capturedPoints)
            map[cp.Y, cp.X] = cp;

    }

    static IList<Point> UpdateAfterMoveMap(Point[,] map, Building myBase)
    {
        //BFS
        var queue = new Stack<Point>();
        queue.Push(myBase);
        var discoveredPoints = new bool[Size, Size];
        discoveredPoints[myBase.Y, myBase.X] = true;

        var activatedPoints = new List<Point>();
        while (queue.Count > 0)
        {
            var p = queue.Pop();
            if (p.Owner == myBase.Owner && !p.IsActive)
            {
                p.IsActive = true;
                activatedPoints.Add(p);
                //Console.Error.WriteLine($"POINT {p.X} {p.Y} IS ACTIVATED");
            }

            if (p.Y > 0)
            {
                var nP = map[p.Y - 1,p.X];
                if (nP != null && nP.Owner == myBase.Owner && !discoveredPoints[nP.Y,nP.X])
                {
                    discoveredPoints[nP.Y, nP.X] = true;
                    queue.Push(nP);
                }
            }
            if (p.Y < Size - 1)
            {
                var nP = map[p.Y + 1,p.X];
                if (nP != null && nP.Owner == myBase.Owner && !discoveredPoints[nP.Y, nP.X])
                {
                    discoveredPoints[nP.Y, nP.X] = true;
                    queue.Push(nP);
                }
            }

            if (p.X > 0)
            {
                var nP = map[p.Y,p.X - 1];
                if (nP != null && nP.Owner == myBase.Owner && !discoveredPoints[nP.Y, nP.X])
                {
                    discoveredPoints[nP.Y, nP.X] = true;
                    queue.Push(nP);
                }
            }

            if (p.X < Size - 1)
            {
                var nP = map[p.Y,p.X + 1];
                if (nP != null && nP.Owner == myBase.Owner && !discoveredPoints[nP.Y, nP.X])
                {
                    discoveredPoints[nP.Y, nP.X] = true;
                    queue.Push(nP);
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
        if (point.X > 0)
            neighbours.Add(map[point.Y, point.X - 1]);
        if (point.X < Size - 1)
            neighbours.Add(map[point.Y, point.X + 1]);
        if (point.Y > 0)
            neighbours.Add(map[point.Y - 1,point.X]);
        if (point.Y < Size - 1)
            neighbours.Add(map[point.Y + 1,point.X]);
        

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
        return map[y,x] != null && map[y,x].Owner == owner && map[y,x] is Building building &&
               building.BuildingType == 2;
    }

    public static bool IsTowerInfluenceCell(int x, int y, Point[,] map, int owner)
    {
        if (map[y,x] == null)
            return false;

        if (IsTowerCell(x, y, map, owner))
            return true;

        if (y > 0)
            if (map[y - 1, x] != null && map[y - 1, x].IsActive && IsTowerCell(x, y - 1, map, owner))
                return true;
        if (y < Size - 1)
            if (map[y + 1, x] != null && map[y + 1, x].IsActive && IsTowerCell(x, y+1, map, owner))
                return true;
        if (x > 0)
            if (map[y, x - 1] != null && map[y, x-1].IsActive && IsTowerCell(x-1, y, map, owner))
                return true;
        if (x < Size - 1)
            if (map[y, x + 1] != null && map[y, x+1].IsActive && IsTowerCell(x + 1, y, map, owner))
                return true;
       
        return false;
    }

    static int[,] GetOppBaseMap (Point[,] map, Point oppBase)
    {
        var resMap = new int[Size, Size];

        var visitedPoints = new bool[Size, Size];
        var queue = new Queue<Tuple<Point, int>>();
        queue.Enqueue(new Tuple<Point, int>(oppBase,0));
        visitedPoints[oppBase.Y, oppBase.X] = true;

        while (queue.Any())
        {
            var item = queue.Dequeue();
            var point = item.Item1;
            resMap[point.Y, point.X] = item.Item2;

            if (point.Y > 0)
            {
                if (map[point.Y - 1, point.X] != null && !visitedPoints[point.Y - 1, point.X])
                {
                    visitedPoints[point.Y - 1, point.X] = true;
                    queue.Enqueue(new Tuple<Point, int>(map[point.Y - 1, point.X], item.Item2 + 1));
                    
                }
            }

            if (point.Y < Size - 1)
            {
                if (map[point.Y + 1, point.X] != null && !visitedPoints[point.Y + 1, point.X])
                {
                    visitedPoints[point.Y + 1, point.X] = true;
                    queue.Enqueue(new Tuple<Point, int>(map[point.Y + 1, point.X], item.Item2 + 1));

                }
            }

            if (point.X > 0)
            {
                if (map[point.Y , point.X - 1] != null && !visitedPoints[point.Y , point.X - 1])
                {
                    visitedPoints[point.Y , point.X - 1] = true;
                    queue.Enqueue(new Tuple<Point, int>(map[point.Y , point.X - 1], item.Item2 + 1));

                }
            }

            if (point.X < Size - 1)
            {
                if (map[point.Y, point.X + 1] != null && !visitedPoints[point.Y, point.X + 1])
                {
                    visitedPoints[point.Y, point.X + 1] = true;
                    queue.Enqueue(new Tuple<Point, int>(map[point.Y, point.X + 1], item.Item2 + 1));

                }
            }

        }

        return resMap;
    }

    static int[,] GetOppBorderMap(Point[,] map)
    {
        var resMap = new int[Size, Size];
        for (var i = 0; i < Size; ++i)
        {
            for (var j = 0; j < Size; ++j)
            {
                var p = map[i, j];
                if (p == null)
                {
                    resMap[i, j] = BIG_WEIGHT;
                    continue;
                }

                if (p.Owner == 1)
                {
                    resMap[i, j] = 0;
                    continue;
                }

                var minDist = int.MaxValue;
                for (var ii = 0; ii < Size; ++ii)
                {
                    for (var jj = 0; jj < Size; ++jj)
                    {
                        var pp = map[ii, jj];
                        if (pp == null || pp.Owner != 1 || !pp.IsActive) continue;
                        var dist = GetManhDist(j, i, jj, ii);
                        if (dist < minDist)
                            minDist = dist;
                    }
                }

                resMap[i, j] = minDist;
            }
        }

        return resMap;
    }

    public static bool[][,] GetOppNeutralMap(Point[,] map, Point myBase, Point oppBase)
    {
        var res = new bool[2][,];

        var stack = new Stack<Point>();
        stack.Push(oppBase);
        var visitedPoints = new bool[Size, Size];

        while (stack.Any())
        {
            var point = stack.Pop();
            visitedPoints[point.Y, point.X] = true;

            if (point.Y > 0)
            {
                if (map[point.Y - 1, point.X] != null && !visitedPoints[point.Y - 1, point.X] && map[point.Y - 1, point.X].Owner != 0)
                {
                    visitedPoints[point.Y - 1, point.X] = true;
                    stack.Push(map[point.Y - 1, point.X]);

                }
            }

            if (point.Y < Size - 1)
            {
                if (map[point.Y + 1, point.X] != null && !visitedPoints[point.Y + 1, point.X] && map[point.Y + 1, point.X].Owner != 0)
                {
                    visitedPoints[point.Y + 1, point.X] = true;
                    stack.Push(map[point.Y + 1, point.X]);

                }
            }

            if (point.X > 0)
            {
                if (map[point.Y, point.X - 1] != null && !visitedPoints[point.Y, point.X - 1] && map[point.Y, point.X-1].Owner != 0)
                {
                    visitedPoints[point.Y, point.X - 1] = true;
                    stack.Push(map[point.Y, point.X - 1]);

                }
            }

            if (point.X < Size - 1)
            {
                if (map[point.Y, point.X + 1] != null && !visitedPoints[point.Y, point.X + 1] && map[point.Y, point.X+1].Owner != 0)
                {
                    visitedPoints[point.Y, point.X + 1] = true;
                    stack.Push(map[point.Y, point.X + 1]);

                }
            }

        }

        res[0] = visitedPoints;


        stack = new Stack<Point>();
        stack.Push(myBase);
        visitedPoints = new bool[Size, Size];

        while (stack.Any())
        {
            var point = stack.Pop();
            visitedPoints[point.Y, point.X] = true;

            if (point.Y > 0)
            {
                if (map[point.Y - 1, point.X] != null && !visitedPoints[point.Y - 1, point.X] && map[point.Y - 1, point.X].Owner != 1)
                {
                    visitedPoints[point.Y - 1, point.X] = true;
                    stack.Push(map[point.Y - 1, point.X]);

                }
            }

            if (point.Y < Size - 1)
            {
                if (map[point.Y + 1, point.X] != null && !visitedPoints[point.Y + 1, point.X] && map[point.Y + 1, point.X].Owner != 1)
                {
                    visitedPoints[point.Y + 1, point.X] = true;
                    stack.Push(map[point.Y + 1, point.X]);

                }
            }

            if (point.X > 0)
            {
                if (map[point.Y, point.X - 1] != null && !visitedPoints[point.Y, point.X - 1] && map[point.Y, point.X - 1].Owner != 1)
                {
                    visitedPoints[point.Y, point.X - 1] = true;
                    stack.Push(map[point.Y, point.X - 1]);

                }
            }

            if (point.X < Size - 1)
            {
                if (map[point.Y, point.X + 1] != null && !visitedPoints[point.Y, point.X + 1] && map[point.Y, point.X + 1].Owner != 1)
                {
                    visitedPoints[point.Y, point.X + 1] = true;
                    stack.Push(map[point.Y, point.X + 1]);

                }
            }

        }

        res[1] = visitedPoints;


        return res;
    }
   
}