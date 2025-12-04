using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DistanceData
{
    public DistanceData(Vector3 _p1, Vector3 _p2, float _distance, int _level)
    {
        p1 = _p1;
        p2 = _p2;
        distance = _distance;
        level = _level;
    }

    public Vector3 p1;
    public Vector3 p2;
    public float distance;
    public int level;
}