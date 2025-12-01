
using System;
using System.IO;
using System.Text;

namespace CsCore
{
    namespace IO
    {
        public class EventLog
        {
            public static string directory = "./eventLog/";

            public static void Write(string text)
            {
                string path = directory + String.Format("{0:0000}-{1:00}-{2:00}.eventLog", DateTime.Now.Year, DateTime.Now.Month, DateTime.Now.Day);

                if (!Directory.Exists(directory))
                {
                    Directory.CreateDirectory(directory);
                }

                //pjh
                // using (var stream =  new FileStream(path, FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None))
                // {
                //     using (var reader = new StreamReader(stream))
                //     using (var writer = new StreamWriter(stream))
                //     {
                //         string date =
                //             $"{DateTime.Now.Year:0000}-{DateTime.Now.Month:00}-{DateTime.Now.Day:00}_{DateTime.Now.Hour:00}:{DateTime.Now.Minute:00}:{DateTime.Now.Second:00}";
                //         string texts = reader.ReadToEnd();
                //         stream.Position = 0; 
                //         string textWrite = date + "     " + text;
                //         writer.WriteLine(textWrite);
                //         writer.Write(texts);
                //     }
                // }

                try
                {
                    // 먼저 파일을 읽기
                    string texts;
                    using (var stream = new FileStream(path, FileMode.OpenOrCreate, FileAccess.Read, FileShare.ReadWrite))
                    {
                        using (var reader = new StreamReader(stream))
                        {
                            texts = reader.ReadToEnd();
                        }
                    }

                    // 파일에 쓸 내용 준비
                    string date =
                        $"{DateTime.Now.Year:0000}-{DateTime.Now.Month:00}-{DateTime.Now.Day:00}_{DateTime.Now.Hour:00}:{DateTime.Now.Minute:00}:{DateTime.Now.Second:00}";
                    string textWrite = date + "     " + text; // text는 외부에서 받은 값이거나 이미 정의된 변수로 가정

                    // 파일에 다시 쓰기 (파일을 쓰기 위해 새로 열기)
                    using (var stream = new FileStream(path, FileMode.OpenOrCreate, FileAccess.Write, FileShare.ReadWrite))
                    {
                        using (var writer = new StreamWriter(stream))
                        {
                            writer.WriteLine(textWrite); // 날짜와 텍스트 쓰기
                            writer.Write(texts); // 기존 텍스트 추가
                        }
                    }
                }
                catch (IOException ex)
                {
                    Console.WriteLine("파일 작업 중 오류 발생: " + ex.Message);
                }
                //~pjh
            }
        }
    }


}