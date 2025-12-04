using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OperationHistoryDaily
{
    public OperationHistoryDaily()
    {
        startHour = 0;
        startMin = 0;
        lastHour = 0;
        lastMin = 0;
    }
    public int startHour;
    public int startMin;
    public int lastHour;
    public int lastMin;
}

public class OperationHistory
{
    public OperationHistory(int pier, int crane, int year, OperationHistoryDaily[] daily)
    {
        this.year = year;
        this.pier = pier;
        this.crane = crane;
        this.daily = daily;
    }

    public int year;
    public int pier;
    public int crane;
    public OperationHistoryDaily[] daily;
}

public class CollisionHistoryDaily
{
    public CollisionHistoryDaily()
    {
        minDistance = new float[24];
        level1 = new char[24];
        level2 = new char[24];
        level3 = new char[24];
    }

    public float[] minDistance;
    public char[] level1;
    public char[] level2;
    public char[] level3;
}

public class CollisionHistoryMonthly
{
    public CollisionHistoryMonthly()
    {
        minDistance = new float[31];
        level1 = new char[31];
        level2 = new char[31];
        level3 = new char[31];
    }

    public float[] minDistance = new float[31];
    public char[] level1 = new char[31];
    public char[] level2 = new char[31];
    public char[] level3 = new char[31];
}

public class CollisionHistory
{
    public CollisionHistory(int pier, int crane, int year, CollisionHistoryDaily[] dayly, CollisionHistoryMonthly[] monthly)
    {
        this.year = year;
        this.pier = pier;
        this.crane = crane;
        this.dayly = dayly;
        this.monthly = monthly;
    }
    public int year;
    public int pier;
    public int crane;
    public CollisionHistoryDaily[] dayly;
    public CollisionHistoryMonthly[] monthly;
}