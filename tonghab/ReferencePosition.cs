using System;
using UnityEngine;

//pjh

[Serializable]
public struct Vector2D_Double
{
    public double x;
    public double y;
    public Vector2D_Double(double _x, double _y)
    {
        x = _x;
        y = _y;
    }
    public static bool operator ==(Vector2D_Double a, Vector2D_Double b)
    {
        return a.Equals(b);
    }
    public static bool operator !=(Vector2D_Double a, Vector2D_Double b)
    {
        return !a.Equals(b);
    }
    public bool Equals(Vector2D_Double other)
    {
        const double epsilon = 1e-10;
        return Math.Abs(x - other.x) < epsilon &&
        Math.Abs(y - other.y) < epsilon;
    }

    public override bool Equals(object obj)
    {
        if(obj is Vector2D_Double other)
        {
            return Equals(other);
        }
        return false;
    }

    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 23 + x.GetHashCode();
            hash = hash * 23 + y.GetHashCode();
            return hash; 
        }
    }
    public double Distance(Vector2D_Double other)
    {
        double dx = x - other.x;
        double dy = y - other.y;
        return Math.Sqrt(dx * dx + dy * dy);
    }
}

public class ReferencePosition : MonoBehaviour
{
    public Vector2D_Double startPoint;
    public Vector2D_Double endPoint;

    [SerializeField] bool drawGizmo;
    [SerializeField] float gizmoRadius = 5f;
    public Transform endPointUnityPosition;

    private void Awake()
    {
        startPoint.x = CoordinateConverter.ConvertToDecimal(startPoint.x.ToString());
        startPoint.y = CoordinateConverter.ConvertToDecimal(startPoint.y.ToString(), false);

        endPoint.x = CoordinateConverter.ConvertToDecimal(endPoint.x.ToString());
        endPoint.y = CoordinateConverter.ConvertToDecimal(endPoint.y.ToString(), false);
    }

    private void OnDrawGizmos()
    {
        if (drawGizmo)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(endPointUnityPosition.position, gizmoRadius);

            Gizmos.color = Color.green;
            Gizmos.DrawSphere(transform.position, gizmoRadius);
        }
    }
}
//~pjh