# system
import time

#ros
import rospy

class SimpleSubscriber(object):
    def __init__(self, topic, messageType, externalCallback=None, rate_limit_hz=None):
        self.topic = topic
        self.messageType = messageType
        self.externalCallback = externalCallback
        self.hasNewMessage = False
        self.lastMsg = None
        self._rate_limit_hz = rate_limit_hz
        self._last_rate_limit_time = time.time()

        if self._rate_limit_hz is not None:
            self._rate_limit_duration_seconds = 1.0/self._rate_limit_hz


    def start(self, queue_size=None):
        self.subscriber = rospy.Subscriber(self.topic, self.messageType, self.callback, queue_size=queue_size)

    def stop(self):
        self.subscriber.unregister()

    def callback(self, msg):
        self.lastMsg = msg
        self.hasNewMessage = True

        if self._rate_limit_hz is not None:
            curr_time = time.time()
            if (curr_time - self._last_rate_limit_time) < self._rate_limit_duration_seconds:
                return
            else:
                self._last_rate_limit_time = curr_time

        if self.externalCallback is not None:
            self.externalCallback(msg)

    def waitForNextMessage(self, sleep_duration=0.1):
        self.hasNewMessage = False
        while not self.hasNewMessage:
            rospy.sleep(sleep_duration)
        return self.lastMsg