import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from fsr_interfaces.action import TriggerFsr
from pyFSRray import FSRmsg_pb2
from pyFSRray.FSRray import FSRray
import matplotlib.pyplot as plt
import os
import numpy as np

class FsrActionServer(Node):

    def __init__(self):
        super().__init__('fsr_action_server')
        self._action_server = ActionServer(
            self,
            TriggerFsr,
            'trigger_fsr',
            self.execute_callback)
        self.show_flag = False
        self.duration = 5
        self.record_flag = False
        self.fsrmsg = FSRmsg_pb2.FSRMsg()
        self.get_logger().info('FSR Action Server has been started.')
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        #values = [0, 1]
        self.show_flag = goal_handle.request.show_flag
        self.duration = goal_handle.request.duration
        self.record_flag = goal_handle.request.record_flag

        goal_handle.succeed()

        self.calibrate(goal_handle)
        mean_layer1, mean_layer2 = self.show_fsrmsg(self.fsrmsg, 'FSR Calibration')

        result = TriggerFsr.Result()
        result.file_name = 'fsr_data.bin'
        return result
    
    def show_fsrmsg(self, fsrmsg, title):
        len_frames = len(fsrmsg.fsr_data)
        all_layer1 = np.zeros((len_frames,16,16), dtype=np.int32)
        all_layer2 = np.zeros((len_frames,16,16), dtype=np.int32)
        for i in range(len_frames):
            data = fsrmsg.fsr_data[i]
            value = data.value
            ts = data.timestamp
            fsr_data = np.fromstring(value, dtype=int, sep=' ')
            fsr_data = fsr_data.reshape(16, 32)
            all_layer1[i] = fsr_data[:, 0:16]
            all_layer2[i] = fsr_data[:, 16:]
        mean_layer1 = all_layer1.mean(axis=0)
        mean_layer2 = all_layer2.mean(axis=0)
        if self.show_flag:
            #vmin = min(np.min(mean_layer1), np.min(mean_layer2))
            #vmax = max(np.max(mean_layer1), np.max(mean_layer2))
            std_layer1 = all_layer1.std(axis=0)
            std_layer2 = all_layer2.std(axis=0)
            s_vmin = min(np.min(std_layer1), np.min(std_layer2))
            s_vmax = max(np.max(std_layer1), np.max(std_layer2))
            fig, axs = plt.subplots(2, 2, figsize=(6, 6))
            pcm = axs[0, 0].imshow(mean_layer1, cmap='viridis', interpolation='none')#, vmin=vmin, vmax=vmax
            fig.colorbar(pcm, ax=axs[0, 0])
            axs[0, 0].set_title('mean layer1')
            pcm = axs[0, 1].imshow(mean_layer2, cmap='viridis', interpolation='none')#, vmin=vmin, vmax=vmax)
            fig.colorbar(pcm, ax=axs[0, 1])
            axs[0, 1].set_title('mean layer2')
            pcm = axs[1, 0].imshow(std_layer1, cmap='viridis', interpolation='none')#, vmin=s_vmin, vmax=s_vmax)
            fig.colorbar(pcm, ax=axs[1, 0])
            axs[1, 0].set_title('std layer1')
            pcm = axs[1, 1].imshow(std_layer2, cmap='viridis', interpolation='none')#, vmin=s_vmin, vmax=s_vmax)
            fig.colorbar(pcm, ax=axs[1, 1])
            axs[1, 1].set_title('std layer2')
            fig.suptitle(title)
            plt.show()
        return mean_layer1, mean_layer2

    def calibrate(self, goal_handle):
        def fsr_callback(values, dt):
            print("dt = {}.{}".format(dt[0], dt[1]))
            #for i in range(16):
            #    print(values[i*16:(i+1)*16])
            max_values = np.max(values)
            min_values = np.min(values)
            feedback_msg = TriggerFsr.Feedback()
            feedback_msg.values = [min_values, max_values]
            fsrdata = FSRmsg_pb2.FSRData()
            fsrdata.timestamp = str(dt[0])
            fsrdata.value = ' '.join(map(str, values))
            #print(fsrdata.__str__())
            self.fsrmsg.fsr_data.append(fsrdata)
            if len(self.fsrmsg.fsr_data) % 20 == 0:
                goal_handle.publish_feedback(feedback_msg)
        self.fsrmsg = FSRmsg_pb2.FSRMsg()
        fsrray = FSRray(16, 2)
        fsrray.set_callback(fsr_callback)
        fsrray.connect()
        time.sleep(self.duration)
        fsrray.disconnect()

def main(args=None):
    rclpy.init(args=args)

    fsr_action_server = FsrActionServer()

    rclpy.spin(fsr_action_server)


if __name__ == '__main__':
    main()
