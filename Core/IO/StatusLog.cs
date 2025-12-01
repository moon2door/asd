using System;
using System.IO;
using System.Text;

namespace CsCore
{
    namespace IO
    {
        public class StatusLog
        {
            public static string directory = "./statusLog/";
            public static bool bIsCreate = false;

            public static void Write(string text, string crane)
            {
                string path = directory + String.Format("{0:0000}-{1:00}-{2:00}_{3}_statusLog.csv", DateTime.Now.Year, DateTime.Now.Month, DateTime.Now.Day, crane);
                FileInfo file = new FileInfo(path);
                if (!file.Exists)
                {
                    bIsCreate = true;
                }

                if (!Directory.Exists(directory))
                {
                    Directory.CreateDirectory(directory);
                }
                
                using (var stream =  new FileStream(path, FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.Read))
                {
                    using (var reader = new StreamReader(stream))
                    using (var writer = new StreamWriter(stream))
                    {
                        string date =
                            String.Format("{0:0000}-{1:00}-{2:00}_{3:00}:{4:00}:{5:00}_{6:000}", DateTime.Now.Year, DateTime.Now.Month, DateTime.Now.Day, DateTime.Now.Hour, DateTime.Now.Minute, DateTime.Now.Second, DateTime.Now.Millisecond);
                        string texts = reader.ReadToEnd();
                        //stream.Position = 0;

                        if (bIsCreate)
                        {
                            writer.WriteLine(" ," + " Rotor1"+ " , , , ," + "Rotor2");
                            writer.WriteLine("DateTime, ErrorCode, RotorFPS, RotorRPM, LidarFPS, ErrorCode, RotorFPS, RotorRPM, LidarFPS");
                            bIsCreate = false;
                        }
                        
                        string textWrite = date + "     " + text;
                        writer.WriteLine(textWrite);
                    }
                }
            }
        }
    }


}