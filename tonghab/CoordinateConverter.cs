using System;

public class CoordinateConverter
{
    public static double ConvertToDecimal(string coordinate, bool isLatitude = true)
    {
        if(coordinate.Length<3) 
        {
            UnityEngine.Debug.LogWarning("latitude or longtitude is 0");
            return 0f;
        }
        // Find the index of the decimal point
        int degreeLength = isLatitude ? 2 : 3;
        // Extract degrees and minutes
        double degrees = double.Parse(coordinate.Substring(0, degreeLength));
        double minutesAndSeconds = double.Parse(coordinate.Substring(degreeLength));

        int minutes = (int)Math.Floor(minutesAndSeconds);
        double seconds = (minutesAndSeconds - minutes) * 60;
        
        var decimalCoordinate = degrees + (minutes / 60f) + (seconds/3600f);

        return decimalCoordinate;
    }

    public static string ConvertToLatLongString(double latitude, double longitude)
    {
        // 위도: N(북위) 또는 S(남위) 결정
        string latDirection = latitude >= 0 ? "N" : "S";
        double absLatitude = Math.Abs(latitude);

        // 경도: E(동경) 또는 W(서경) 결정
        string lonDirection = longitude >= 0 ? "E" : "W";
        double absLongitude = Math.Abs(longitude);

        // 위도 변환
        int latDegrees = (int)Math.Floor(absLatitude);
        double latFractional = (absLatitude - latDegrees) * 60;
        int latMinutes = (int)Math.Floor(latFractional);
        double latSeconds = (latFractional - latMinutes) * 60;

        // 경도 변환
        int lonDegrees = (int)Math.Floor(absLongitude);
        double lonFractional = (absLongitude - lonDegrees) * 60;
        int lonMinutes = (int)Math.Floor(lonFractional);
        double lonSeconds = (lonFractional - lonMinutes) * 60;

        // 위도와 경도를 N/S, E/W 형식으로 반환
        string formattedLatitude = $"{latDegrees}° {latMinutes}' {latSeconds:F2}\" {latDirection}";
        string formattedLongitude = $"{lonDegrees}° {lonMinutes}' {lonSeconds:F2}\" {lonDirection}";

        return $"{formattedLatitude}, {formattedLongitude}";
    }
}