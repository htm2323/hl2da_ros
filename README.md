# HoloLens2センサーデータROS2配信アプリ
HoloLens2のセンサーデータをROS2で外部へ配信するUnityプロジェクトです．
[hl2da](https://github.com/jdibenes/hl2da)を使用してセンサにアクセスしています．
このリポジトリには，深度センサのデータをROS2で配信するUWPアプリケーションと，配信された深度画像をサブスクライブするROS2パッケージを含みます．

## 動作確認環境
- **HoloLens2アプリ**
  - Unity 2022.3.51f1
  - [hl2da](https://github.com/jdibenes/hl2da)
  - [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
  - MRTK3
    - MRTK Core Definitions 4.0.0-pre.1
    - MRTK Diagnostics
    - MRTK Input 4.0.0-pre.1
    - MRTK Spatial Manipulation 4.0.0-pre.1
    - MRTK UX components 4.0.0-pre.1
    - MRTK UX components 4.0.0-pre.1
    - MRTK UX Core Scripts 4.0.0-pre.1
    - Mixed Reality OpenXR Plugin 1.11.1
    - Microsoft Spatializer 2.0.55

- **ROS2パッケージ**
  - Ubuntu 20.04
  - ROS2 foxy
  - [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)

## 導入(HoloLens2アプリ)
1. このリポジトリをクローンする
2. Unityで`unity`フォルダ内にあるプロジェクトを開く（SafeModeに入る）
3. Mixed Reality Feature Toolで **MRTK3の各種パッケージ** をimport
4. パッケージマネージャーから**ROS-TCP-Connector**をimport
5. Exit SafeModeする
6. MRTK3の設定を行う
   - Build Settingsから**platform を UWP に**し，**Architecture を ARM 64bit に**変更
   - ProjectSettingsを開く
     - MRTK3 タブで **Assign MRTK Default** を押してプロファイルを設定
     - XR Plug-in Managementタブで **OpenXR と Microsoft HoloLens feature group** を選択し，HoloLens2向けビルド設定を行う
     - PlayerタブでPublishing Settingsを開き，**Certificcateファイルを作成**する
7. SampleSceneを開き，TextMeshProをimport
8. RoboticsタブからROSの設定を開き，ROS-TCP-Endpointを起動している端末のIPアドレスを記入する
9. ビルド

## 導入(ROS2パッケージ)
1. このリポジトリをクローンする
2. `colcon build --packages-select hl2da_ros`を実行
3. ROS-TCP-Endpointを実行しておく
4. `ros2 run hl2da_ros depth_subscriber`でノードが起動します
