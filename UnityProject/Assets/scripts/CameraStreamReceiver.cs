using System;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

public class CameraStreamReceiver : MonoBehaviour
{
    public RawImage displayImage;
    public string host = "127.0.0.1";
    public int port = 55002;

    private TcpClient client;
    private Thread receiveThread;
    private Texture2D texture;
    private byte[] pendingFrame;
    private bool hasNewFrame = false;
    private readonly object frameLock = new object();

    void Start()
    {
        texture = new Texture2D(2, 2);
        receiveThread = new Thread(ReceiveFrames) { IsBackground = true };
        receiveThread.Start();
    }

void ReceiveFrames()
{
    while (true)   // blijft proberen na elke disconnect
    {
        try
        {
            client = new TcpClient(host, port);
            NetworkStream stream = client.GetStream();
            byte[] header = new byte[4];

            while (true)
            {
                ReadExact(stream, header, 4);
                int size = (header[0] << 24) | (header[1] << 16) | (header[2] << 8) | header[3];
                byte[] frameData = new byte[size];
                ReadExact(stream, frameData, size);

                lock (frameLock)
                {
                    pendingFrame = frameData;
                    hasNewFrame = true;
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[Camera] {e.Message} — opnieuw verbinden in 2s...");
            client?.Close();
            Thread.Sleep(2000);
        }
    }
}

    void ReadExact(NetworkStream stream, byte[] buf, int count)
    {
        int received = 0;
        while (received < count)
            received += stream.Read(buf, received, count - received);
    }

    void Update()
    {
        lock (frameLock)
        {
            if (!hasNewFrame) return;
            texture.LoadImage(pendingFrame);
            displayImage.texture = texture;
            hasNewFrame = false;
        }
    }

    void OnDestroy()
    {
        receiveThread?.Abort();
        client?.Close();
    }
}
