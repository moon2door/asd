namespace PortSystem.Data // 혹은 그냥 Data
{

    public struct StRotorInfo
    {
        public int lidarFps;
        public int rotorFps;
        public float rotorRpm;
        public bool packetError;
        public bool rotationError;
        public bool proxyMeterError;
        public bool zeroSetError;
        public bool encoderError;
        public bool NetworkError;
        public byte errorCodeActurator;
    }
}