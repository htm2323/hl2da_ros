using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System;

public class HoloLensDepthPublisher : MonoBehaviour
{
    ROSConnection ros;
    [SerializeField] private string depthTopicName = "hololens/depth";
    [SerializeField] private string frame_id = "Hololens";

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<CompressedImageMsg>(depthTopicName);
    }

    public void PublishMessage(byte[] image)
    {
        DateTimeOffset dto = new DateTimeOffset(DateTime.Now);
        TimeMsg timestamp = new TimeMsg { sec = (int)dto.ToUnixTimeSeconds(), nanosec = (uint)(dto.ToUnixTimeMilliseconds() % 1000 * 1E6) };

        HeaderMsg header = new HeaderMsg(timestamp, frame_id);

        CompressedImageMsg msg = new CompressedImageMsg(
            header,
            "PNG",
            image
        );

        ros.Publish(depthTopicName, msg);

        //Debug.Log("publishing message");
    }
}