import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

class DepthSubscriber(Node):
    """Hololensから深度画像を受け取るためのノード"""

    def __init__(self):
        super().__init__('depth_subscriber')

        self.bridge = CvBridge()

        self.save_path = 'depthimage/'

        # fps計測
        self.frame_count = 0
        self.measurement_count = 10
        self.before_frame = 0
        self.fps = 0
        self.tm = cv2.TickMeter()
        self.tm.start()

        # QoS Settings
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        depth_subscriber = self.create_subscription(
            CompressedImage,
            '/hololens/depth',
            self.callback,
            qos_profile=qos
        )

    def callback(self, depth_src:CompressedImage):
        # FPSカウントアップ
        self.frame_count_up()

        buf = np.ndarray(shape=(1, len(depth_src.data)),
                        dtype=np.uint8, buffer=depth_src.data)
        depth_img: np.ndarray = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)

        # hl2da aquire flip image, so flip it back
        depth_img = cv2.flip(depth_img, 0)

        # HololensがAHATモードの場合、1m以上の深度値は4095の外れ値で置き換えられているため、丸め込む。外れ値以外の最大値は1055
        depth_img = np.where(depth_img > 4000, 1055, depth_img)

        self.save_image(depth_img)
        self.preview_image(depth_img)

    def save_image(self, img):
        """
        subscribeした画像を保存します.
        コールバックが呼ばれたときに呼び出す必要があります.
        """
        timestr = datetime.datetime.now().strftime('%Y_%m_%d-%H_%M_%S_%f')
        file_path = os.path.join(self.save_path, timestr + '.png')
        cv2.imwrite(file_path, img)

    def preview_image(self, img):
        """
        subscribeした画像を表示します.
        コールバックが呼ばれたときに呼び出す必要があります.
        """
        # 8bit化して表示
        img = img.astype(np.float32)
        img = (img/1055)* 256
        
        img = img.astype(np.uint8)

        img = cv2.applyColorMap(img, cv2.COLORMAP_TURBO)

        # FPS印字
        img = self.print_fps(img)
        
        # add color map sample
        color_bar= cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(1, -1), cv2.COLORMAP_TURBO)
        color_bar = cv2.resize(color_bar, (img.shape[1], 30))

        result_img = np.vstack((img, color_bar))
        
        cv2.namedWindow('Preview Depth', cv2.WINDOW_NORMAL)
        cv2.imshow('Preview Depth', result_img)
        cv2.waitKey(1)


    def frame_count_up(self):
        """
        fpsを計算するためのフレーム計算をします.
        コールバックが呼ばれたときに呼び出す必要があります.

        :return:
        """
        self.frame_count += 1
        # fps計算
        if self.frame_count % self.measurement_count == 0:
            self.tm.stop()
            self.fps = (self.frame_count - self.before_frame) / self.tm.getTimeSec()
            self.before_frame = self.frame_count
            self.tm.reset()
            self.tm.start()

    def print_fps(self, src: np.ndarray) -> np.ndarray:
        """
        fpsを画像に印字します.

        :param src:
        :return:
        """
        img = src

        if src.ndim == 2:
            # 2次元 -> モノクロ画像
            img = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)

        cv2.putText(img, "frame = " + str(self.frame_count), (0, 20), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0))
        cv2.putText(img, 'FPS: {:.2f}'.format(self.fps),
                    (0, 40), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0))

        return img

def main(args=None):
    rclpy.init(args=args)

    depth_subscriber = DepthSubscriber()

    try:
        rclpy.spin(depth_subscriber)

    except KeyboardInterrupt:
        pass

    finally:
        print()
        # 終了処理
        depth_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

