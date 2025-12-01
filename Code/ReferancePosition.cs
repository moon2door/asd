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
public class ReferancePosition : MonoBehaviour
{
    public bool gpsData = false;
    public Vector2D_Double startPoint;
    public Vector2D_Double endPoint;

    [SerializeField] bool drawGizmo;
    [SerializeField] float gizmoRadius = 5f;
    public Transform endPointUnityPosition;

    private void Awake()
    {
        // Debug.Log(CoordinateConverter.ConvertToLatLongString(startPoint.x, startPoint.y));
        // Debug.Log(CoordinateConverter.ConvertToLatLongString(endPoint.x, endPoint.y));
        // return;
        if (gpsData == false)
        {
            startPoint.x = CoordinateConverter.ConvertToDecimal(startPoint.x.ToString());
            startPoint.y = CoordinateConverter.ConvertToDecimal(startPoint.y.ToString(), false);

            endPoint.x = CoordinateConverter.ConvertToDecimal(endPoint.x.ToString());
            endPoint.y = CoordinateConverter.ConvertToDecimal(endPoint.y.ToString(), false);
        }
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
    
    // Config 값을 통한 크레인 레인 범위 수정.
    public void BeginStartPosition(string pier)
    {
        // Config로 설정된 pier
        string textPier = CsCore.Configuration.ReadConfigIni("CraneType", "Pier", "ENI");
        if(string.Compare(pier, textPier) == 0){
            // Config 값을 통한 시작 포인트 수정.
            string limitStart_x = CsCore.Configuration.ReadConfigIni("CranePierPosition", "Start_X", "3453.899902");
            string limitStart_y = CsCore.Configuration.ReadConfigIni("CranePierPosition", "Start_Y", "12836.488281");
            string limitEnd_x = CsCore.Configuration.ReadConfigIni("CranePierPosition", "End_X", "3454.030029");
            string limitEnd_y = CsCore.Configuration.ReadConfigIni("CranePierPosition", "End_Y", "12836.511719");

            double start_x = 0;
            double start_y = 0;
            double end_x = 0;
            double end_y = 0;
            if (
                double.TryParse(limitStart_x, out start_x) && double.TryParse(limitStart_y, out start_y) &&
                double.TryParse(limitEnd_x, out end_x) && double.TryParse(limitEnd_y, out end_y)
                )
            {
                startPoint.x = start_x;
                startPoint.y = start_y;
                endPoint.x = end_x;
                endPoint.y = end_y;
            }
        }
    }
}
//~pjh