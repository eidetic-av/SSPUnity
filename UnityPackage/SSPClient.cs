using System;
using System.Collections.Generic;
using System.Reflection;
using System.Threading;
using System.Runtime.InteropServices;
using UnityEngine;
using static UnityEngine.Experimental.Rendering.GraphicsFormat;

using Eidetic.URack;
using Eidetic.URack.Collection;

public class SSPClient : MonoBehaviour
{
    public bool DebugLog = true;
    public string LutName = "NFOV_UNBINNED";

    [DllImport("ssp_client_plugin")]
    static extern void InitNetworkReader(int port);

    [DllImport("ssp_client_plugin")]
    static extern void Close();

    [DllImport("ssp_client_plugin")]
    static extern bool ReaderHasNextFrame();

    [DllImport("ssp_client_plugin")]
    static extern IntPtr GetNextFramePtr(out ulong frameAge);

    const int FrameSize = 640 * 576;

    Thread ReaderThread;
    object ThreadLock = new object();
    bool ClientActive;
    bool UpdateFrame;
    ComputeBuffer DepthBuffer;
    ComputeBuffer Depth2DTo3DBuffer;
    ComputeShader TransferShader;

    public Vector2 ThresholdX = new Vector2(-10f, 10f);
    public Vector2 ThresholdY = new Vector2(-10f, 10f);
    public Vector2 ThresholdZ = new Vector2(0, 12f);

    public PointCloud PointCloud;
    public UnityEngine.UI.RawImage RawImage;

    public Populator populator;
    public Billboard billboard;

    public bool DispatchShaderUpdate = false;
    
    void Start()
    {
        InitNetworkReader(9999);
        Debug("Initialised Network Reader");
        
        TransferShader = Resources.Load("SSPPointCloud") as ComputeShader;
        PointCloud = PointCloud.CreateInstance();

        // depthbuffer length is halved because each 32-bit entry holds two shorts
        DepthBuffer = new ComputeBuffer(FrameSize / 2, 4);

        var lutAsset = Resources.Load(LutName) as TextAsset;
        if (lutAsset == null)
        {
            Debug("Unable to load 2D to 3D lookup table.");
            return;
        }
        Debug($"Loaded lookup table asset {LutName}.");

        var lookupTable = new List<Vector2>();
        var lutString = lutAsset.text;
        var lutEntries = lutString.Split(" ".ToCharArray(), StringSplitOptions.RemoveEmptyEntries);
        var i = 0;

        foreach(var entry in lutEntries)
        {
            var vector = entry.Split(",".ToCharArray());
            if (vector.Length < 2) continue;

            var x = float.Parse(vector[0]);
            var y = float.Parse(vector[1]);

            lookupTable.Add(new Vector2(x, y));
            i++;
        }

        Debug($"Added {i} entries into the lookup table.");

        Depth2DTo3DBuffer = new ComputeBuffer(FrameSize, sizeof(float) * 2);
        Depth2DTo3DBuffer.SetData(lookupTable);
        
        TransferShader.SetBuffer(0, "Depth2DTo3DBuffer", Depth2DTo3DBuffer);

        // send test image to nreal controller
        var controllerPanel = GameObject.Find("BaseControllerPanel");
        if (controllerPanel != null)
        {
            var imageObject = new GameObject("TestImage");
            imageObject.transform.parent = controllerPanel.transform;
            imageObject.transform.localPosition = Vector3.zero;
            RawImage = imageObject.AddComponent<UnityEngine.UI.RawImage>();
            RawImage.transform.localScale = new Vector3(10, 10, 1);
        }


        var sensorThread = new System.Threading.Thread(CheckForSensorFrame);
        ClientActive = true;
        sensorThread.Start();
    }

    void OnDestroy()
    {
        DepthBuffer.Release();
        ClientActive = false;
        Close();
    }

    void CheckForSensorFrame()
    {
        while(ClientActive)
        {
            if (DispatchShaderUpdate) continue;
            // ReaderHasNextFrame is blocking, so it needs to be on a thread
            if (ReaderHasNextFrame())
            {
                Debug("Received frame");
                DispatchShaderUpdate = true;
            }
        }
    }

    void Update()
    {
        if (DispatchShaderUpdate)
        {
            // Set the depth buffer from the plugin memory location
            SetUnmanagedData(DepthBuffer, GetNextFramePtr(out ulong frameAge), FrameSize / 2, 4);
            // if (frameAge > 500)
            // {
            //     Debug($"Frame was {frameAge}ms old, discard it");
            //     return;
            // }
            
            Debug("Successfully set unmanaged data to frame pointer");

            int width = 640;
            int height = 576;
            
            // allocate the data for the GPU
            TransferShader.SetInt("Width", width);
            TransferShader.SetInt("Height", height);
            TransferShader.SetVector("ThresholdX", ThresholdX); 
            TransferShader.SetVector("ThresholdY", ThresholdY); 
            TransferShader.SetVector("ThresholdZ", ThresholdZ); 
            TransferShader.SetBuffer(0, "DepthBuffer", DepthBuffer);

            // create the output textures
            var positionsRt = new RenderTexture(width, height, 24, R32G32B32A32_SFloat);
            positionsRt.enableRandomWrite = true;
            positionsRt.Create();
            TransferShader.SetTexture(0, "Positions", positionsRt, 0);

            var colorsRt = new RenderTexture(width, height, 24, R32G32B32A32_SFloat);
            colorsRt.enableRandomWrite = true;
            colorsRt.Create();
            TransferShader.SetTexture(0, "Colors", colorsRt, 0);
        
            // dispatch the shader
            int gfxThreadWidth = (width * height) / 2 / 64;
            TransferShader.Dispatch(0, gfxThreadWidth, 1, 1);

            // set the result to the pointcloud
            PointCloud.SetPositionMap(positionsRt);
            PointCloud.SetColorMap(colorsRt);

            // if (RawImage != null)
            //     RawImage.texture = PointCloud.PositionMap;

            if (populator == null)
            {
                var positionOffset =  new Vector3(0, .25f, 1f);
                populator = UModule.Create("Populator", 1) as Populator;
                populator.gameObject.transform.position = positionOffset;
                // billboard = UModule.Create("Billboard", 2) as Billboard;
                // billboard.gameObject.transform.position = positionOffset;
            }
            else
            {
                populator.PointCloudInput = PointCloud;
                // billboard.PointCloudInput = PointCloud;
            }

            // cleanup
            Destroy(positionsRt);
            DispatchShaderUpdate = false;
        }
    }

    void Debug(string message)
    {
        if (DebugLog) UnityEngine.Debug.Log("SSP_DEBUG: " + message);
    }

    static MethodInfo SetNativeMethod;
    static object [] MethodArgs = new object[5];

    public static void SetUnmanagedData
        (ComputeBuffer buffer, IntPtr pointer, int count, int stride, int srcOffset = 0, int bufferOffset =0)
    {
        if (SetNativeMethod == null)
        {
            SetNativeMethod = typeof(ComputeBuffer).GetMethod(
                "InternalSetNativeData",
                BindingFlags.InvokeMethod |
                BindingFlags.NonPublic |
                BindingFlags.Instance
            );
        }

        MethodArgs[0] = pointer;
        MethodArgs[1] = srcOffset;
        MethodArgs[2] = bufferOffset;
        MethodArgs[3] = count;
        MethodArgs[4] = stride;

        SetNativeMethod.Invoke(buffer, MethodArgs);
    }
}
