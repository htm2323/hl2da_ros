using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class HoloLensAquiredDepthPreviewController : MonoBehaviour
{
    // plane object for preview depth image
    public GameObject _depth_image;
    public GameObject _depth_color_image;

    [SerializeField] TextMeshPro _fps_text;
    private float timeElapsed;

    public TextMeshPro calibration_text;

    private void Update()
    {
        timeElapsed += Time.deltaTime;
    }

    public void InitializePreview_AHAT(Texture2D tex_grayscale, RenderTexture tex_color)
    {
        _depth_image.GetComponent<MeshRenderer>().material.mainTexture = tex_grayscale;
        _depth_color_image.GetComponent<MeshRenderer>().material.mainTexture = tex_color;
    }

    public void InitializePreview_LongThrow(Texture2D tex_grayscale, RenderTexture tex_color)
    {
        _depth_image.GetComponent<MeshRenderer>().material.mainTexture = tex_color;
        _depth_color_image.GetComponent<MeshRenderer>().material.mainTexture = tex_color;
    }

    public void Update_Preview()
    {

    }

    public void UpdateFPSText()
    {
        _fps_text.text = "FPS : " + (1f / timeElapsed).ToString();
        timeElapsed = 0;
    }

    public void PreviewCalibration(string calibration_data)
    {
        calibration_text.text = calibration_data;
    }
}
