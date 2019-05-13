import cv2
import subprocess as sp
import numpy

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

FFMPEG_BIN = "ffmpeg"
command = [ FFMPEG_BIN,
        '-i', 'fifo',             # fifo is the named pipe
        '-pix_fmt', 'bgr24',      # opencv requires bgr24 pixel format.
        '-vcodec', 'rawvideo',
        '-an','-sn',              # we want to disable audio processing (there is no audio)
        '-f', 'image2pipe', '-']
pipe = sp.Popen(command, stdout = sp.PIPE, bufsize=10**8)
print command
pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
rospy.init_node('CameraStream', anonymous=True)
rate = rospy.Rate(60) # 10hz
br = CvBridge()
i = 0
while True:
    # Capture frame-by-frame
    raw_image = pipe.stdout.read(640*480*3)
    # transform the byte read into a numpy array
    image =  numpy.fromstring(raw_image, dtype='uint8')
    image = image.reshape((480,640,3))          # Notice how height is specified first and then width

    #####################
    import matplotlib
    # i += 1
    # if i%10 == 0:
    #     cv2.imwrite('./images/name'+str(i)+'.png', image)
    #####################
    image_message = br.cv2_to_imgmsg(image, "bgr8")
    rospy.loginfo("Publish image")
    pub.publish(image_message)
    rate.sleep()

#    hello_str = "hello world %s" % rospy.get_time()
#    rospy.loginfo(hello_str)
#    pub.publish(hello_str)
#    rate.sleep()


    if image is not None:
        cv2.imshow('Video', image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    pipe.stdout.flush()

cv2.destroyAllWindows()

