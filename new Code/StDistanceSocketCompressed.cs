namespace PortSystem.Data // 혹은 그냥 Data
{

    public class StDistanceSocketCompressed
    {
        public static int type = 0x01;
        public static int code = 0x0e;

        public StDistanceSocketCompressed(byte[] buffer)
        {
            this.buffer = buffer;
        }

        public byte[] buffer;
    }
}