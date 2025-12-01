
using System;
using System.IO;
using System.Text;
using UnityEngine.Android;
using System.Threading.Tasks;//pjh

namespace CsCore
{
    namespace IO
    {
        public class DistanceLog
        {
            public static string directory = "./distanceLog/";

            public static int reserveDiskGB = 5;
            //pjh
            //public static long GetAvailableFreeSpaceGB()
            public static async Task<long> GetAvailableFreeSpaceGB()
            {
                return await Task.Run(() =>
                {
            //~pjh
                string dataPath = System.Environment.CurrentDirectory;
                string[] pathSplit = dataPath.Split('\\');
                long availableSpace = long.MaxValue;
                if (pathSplit.Length > 0)
                {
                    DriveInfo[] infos = DriveInfo.GetDrives();
                    foreach (DriveInfo info in infos)
                    {
                        if (info.Name.StartsWith(pathSplit[0]))
                        {
                            availableSpace = (long)(info.AvailableFreeSpace / (float)(1024 * 1024 * 1024));
                        }
                    }
                }
                
                return availableSpace;
                });//pjh
            }

            //pjh
            //public static void Write(byte[] data, int length)
            public static async Task Write(byte[] data, int length)
            //~pjh
            {
                Data.CollisionProcessor.StDistanceSocket temp = new Data.CollisionProcessor.StDistanceSocket(data);
                var pier = "G2";
                var crane = "LLC12";
                switch (temp.attitude.pierId)
                {
                    case 4:
                        pier = "Pier6";
                        break;

                    case 5:
                        pier = "G2Dock";
                        break;

                    case 6:
                        pier = "G3Dock";
                        break;

                    case 7:
                        pier = "G4Dock";
                        break;
                    case 8:
                        pier = "Pier0D";
                        break;
                    case 9:
                        pier = "ENIDock";
                        break;
                    default:
                        break;
                }
                if (temp.attitude.pierId == 5)
                {
                    switch (temp.attitude.craneId)
                    {
                        case 0:
                            crane = "LLC12";
                            break;
                        case 1:
                            crane = "LLC13";
                            break;
                        default:
                            break;
                    }
                }
                if (temp.attitude.pierId == 6)
                {
                    switch (temp.attitude.craneId)
                    {
                        case 0:
                            crane = "LLC19";
                            break;
                        case 1:
                            crane = "LLC20";
                            break;
                        default:
                            break;
                    }
                }

                if (temp.attitude.pierId == 7)
                {
                    switch (temp.attitude.craneId)
                    {
                        case 0:
                            crane = "LLC25";
                            break;
                        case 1:
                            crane = "LLC26";
                            break;
                        default:
                            break;
                    }
                }

                //pjh
                if(temp.attitude.pierId == 8)
                {
                    switch(temp.attitude.craneId)
                    {
                        case 0:
                        crane = "LLC16";
                        break;
                    }
                }
                //~pjh

                if (temp.attitude.pierId == 9)
                {
                    switch (temp.attitude.craneId)
                    {
                        case 0:
                            crane = "TC1";
                            break;
                        case 1:
                            crane = "JIB1";
                            break;
                        case 2:
                            crane = "JIB2";
                            break;
                        default:
                            break;
                    }
                }

                string path = directory + String.Format("{0}-{1:00}-{2:00}_{3:00}-{4:00}_{5:00}_{6:00}.distance", DateTime.Now.Year, DateTime.Now.Month, DateTime.Now.Day, DateTime.Now.Hour, (DateTime.Now.Minute / 10) * 10, pier, crane);
                
                if (!Directory.Exists(directory))
                {
                    Directory.CreateDirectory(directory);
                }

                // Remove files
                if (await GetAvailableFreeSpaceGB() <= reserveDiskGB)
                {
                    //pjh
                    string[] files = await Task.Run(()=>
                        Directory.GetFiles(directory));
                    if(files.Length > 0)
                    {
                        await Task.Run(()=>
                            File.Delete(files[0]));
                    }
                    // string[] files = Directory.GetFiles(directory);
                    // if(files.Length > 0)
                    // {
                    //     File.Delete(files[0]);
                    // }
                    //~pjh
                }

                // Write data
                byte[] dummy = new byte[77];
                byte[] header = BitConverter.GetBytes(length);

                if (header.Length >= 4)
                {
                    //pjh
                    //FileStream stream = new FileStream(path, FileMode.Append);
                    FileStream stream = new FileStream(path, FileMode.Append, FileAccess.Write, FileShare.None, bufferSize : 4096);
                    //~pjh
                    byte[] year = BitConverter.GetBytes(DateTime.Now.Year);
                    byte[] month = BitConverter.GetBytes(DateTime.Now.Month);
                    byte[] day = BitConverter.GetBytes(DateTime.Now.Day);
                    byte[] hour = BitConverter.GetBytes(DateTime.Now.Hour);
                    byte[] minute = BitConverter.GetBytes(DateTime.Now.Minute);
                    byte[] sec = BitConverter.GetBytes(DateTime.Now.Second);
                    byte[] wday = BitConverter.GetBytes((int)(DateTime.Now.DayOfWeek));
                    byte[] yday = BitConverter.GetBytes(DateTime.Now.DayOfYear);

                    Array.Copy(sec, 0, dummy, 0, 4);
                    Array.Copy(minute, 0, dummy, 4, 4);
                    Array.Copy(hour, 0, dummy, 8, 4);
                    Array.Copy(day, 0, dummy, 12, 4);
                    Array.Copy(month, 0, dummy, 16, 4);
                    Array.Copy(year, 0, dummy, 20, 4);
                    Array.Copy(wday, 0, dummy, 24, 4);
                    Array.Copy(yday, 0, dummy, 28, 4);
                    
                    //pjh
                    await stream.WriteAsync(dummy, 0, 77);
                    await stream.WriteAsync(header, 0, 4);
                    await stream.WriteAsync(data, 0, length);
                    await stream.FlushAsync();
                    // stream.Write(dummy, 0, 77);
                    // stream.Write(header, 0, 4);
                    // stream.Write(data, 0, length);
                    // stream.Flush();
                    // stream.Close();
                    //~pjh
                }
            }
        }
    }
}
