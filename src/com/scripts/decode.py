#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, String
import threading
import sys
sys.path.append('/home/nvidia/swarm/devel/lib/python3/dist-packages')
from cam.msg import TrackStatus, Decoded

class DecoderNode:
    def __init__(self, base_frequency=1, signal_length=11, repeat_count=3):
        """
        初始化解码器节点
        :param tracker_id: 跟踪器ID (1-4)
        :param base_frequency: 基础频率
        :param signal_length: 信号长度
        :param repeat_count: 重复次数
        """
        # 初始化 ROS 节点
        rospy.init_node(f'signal_decoder', anonymous=True)
        
        # 订阅 tracker 话题
        self.tracker_sub = rospy.Subscriber(f'/unified_tracker', TrackStatus, self.tracker_callback)
        
        # 发布解码后的消息话题
        self.decoded_pub = rospy.Publisher(f'/decoded', Decoded, queue_size=10)
        
        # 信号参数
        self.base_frequency = base_frequency
        self.signal_length = signal_length
        self.repeat_count = repeat_count
        self.packet = (self.base_frequency * self.signal_length * self.repeat_count)*3 + 8
        self.pattern = [1, 1, 0]  # 需要检测的模式


        # 初始化状态
        self.tracking_states = {}  # 用于存储每个 tracking_id 的状态
        self.lock = threading.Lock()  # 用于线程安全的锁

        rospy.loginfo(f"Initialized decoder for tracker")

    def tracker_callback(self, data):
        """
        订阅 tracker 话题的回调函数。
        消息格式为：ID: {tracking_id}, Message: {received_message}
        """
        # 解析接收到的消息

        try:
            tracking_id = data.track_id
            received_bit = data.status
            x_pos = data.x
            y_pos = data.y
            rospy.loginfo(f"Received bit - ID: {tracking_id}, Bit: {received_bit}, Position: ({x_pos}, {y_pos})")
            
            # 将接收到的数据位转换为整数
            bit = int(received_bit)
            
            # 初始化或获取当前 tracking_id 的状态
            with self.lock:
                if tracking_id not in self.tracking_states:
                    self.tracking_states[tracking_id] = {
                        'buffer': [],  # 用于存储接收到的二进制数据流
                        'is_collecting': False,  # 是否正在收集数据
                        'last_x': x_pos,  # 存储最后的x坐标
                        'last_y': y_pos   # 存储最后的y坐标
                    }
                state = self.tracking_states[tracking_id]
                
                # 更新位置信息
                state['last_x'] = x_pos
                state['last_y'] = y_pos
                
                # 将新数据位添加到缓冲区
                state['buffer'].append(bit)
                
                # 如果缓冲区长度足够，检查是否匹配模式
                if len(state['buffer']) >= len(self.pattern) and not state['is_collecting']:
                    # 使用滑动窗口检查是否匹配模式
                    window = state['buffer'][-len(self.pattern):]  # 取最后 pattern_len 个数据位
                    if window == self.pattern:
                        # 检测到完整模式 110，开始收集数据
                        state['is_collecting'] = True
                        
                        # 保留最后一位数据
                        last_bit = state['buffer'][-1]  # 获取最后一位数据
                        state['buffer'] = [last_bit]  # 清空缓冲区，但保留最后一位数据
                        
                        rospy.loginfo(f"ID: {tracking_id} - Pattern 110 detected. Start collecting data.")
                
                # 如果正在收集数据
                if state['is_collecting']:
                    # rospy.loginfo(f"Received bit - ID: {tracking_id}, Bit: {received_bit}")
            
                    # 检查是否收集到足够的数据
                    if len(state['buffer']) >= self.packet:
                        # 提取信号并解码
                        signal = state['buffer'][:self.packet]
                        rospy.loginfo(f"ID: {tracking_id} - Decoding signal: {signal}")
                        
                        # 重置状态
                        state['is_collecting'] = False
                        state['buffer'] = []
                        
                        # 启动新线程进行解码
                        threading.Thread(
                            target=self.decode_and_publish,
                            args=(tracking_id, signal, state['last_x'], state['last_y']),
                            daemon=True
                        ).start()

        except Exception as e:
            rospy.logerr(f"Error processing message: {e}")

    def decode_and_publish(self, tracking_id, signal, last_x, last_y):
        """
        解码信号并发布结果。
        """
        decoded_messages = self.decode_signal(signal)
        if decoded_messages:
            # 检查解码后的消息列表
            if len(decoded_messages) == 3:
                # 检查三个值是否都不同
                if len(set(decoded_messages)) == 3:
                    rospy.loginfo("All three values are different. Decoding failed.")
                else:
                    # 使用投票机制决定发布的值
                    from collections import Counter
                    counter = Counter(decoded_messages)
                    most_common_value = counter.most_common(1)[0][0]
                    
                    # 发布解码后的消息
                    with self.lock:
                        decoded_msg = Decoded()
                        decoded_msg.command = most_common_value
                        decoded_msg.x = last_x
                        decoded_msg.y = last_y
                        self.decoded_pub.publish(decoded_msg)
                        rospy.loginfo(f"ID: {tracking_id} - Published decoded message: {most_common_value}, Position: ({last_x}, {last_y})")
            else:
                rospy.logwarn(f"Decoded messages count is not 3: {decoded_messages}")
        else:
            rospy.logwarn("No valid decoded messages.")

    def preprocess_signal(self, signal):
        """完整的信号预处理流程：
        1. 先聚类
        2. 对第一个簇进行特殊拆分（前5个单独拆分）
        3. 对所有簇进行常规过采样处理
        """
        # 第一步：初始聚类
        clusters = self.cluster_samples(signal)  
        
        # 第二步：特殊处理第一个簇
        clusters = self.split_first_cluster(clusters)
        
        # 第三步：常规过采样处理（包括拆分后的簇）
        processed_clusters = self.split_large_clusters(clusters)
                # 生成理想化采样序列
        ideal_samples = [val for val, cnt in processed_clusters if cnt > 1]
        return ideal_samples
    
    def split_first_cluster(self, clusters):
        """特殊处理第一个簇：
        - 如果样本数>5则拆分为[前5个, 剩余部分]
        - 否则保持原样
        """
        if not clusters or len(clusters[0]) != 2:  # 安全校验
            return clusters
            
        first_val, first_count = clusters[0]
        
        if first_count > 4:
            return [
                (first_val, 3),
                (first_val, first_count - 5),
                *clusters[1:]  # 保留其他簇
            ]
        return clusters

    def clusters_to_signal(self, clusters):
        """将聚类结果转换回信号序列"""
        signal = []
        for value, count in clusters:
            signal += [value] * count
        return signal

    # 修改后的解码方法
    def decode_signal(self, signal):
        """解码信号（添加预处理）"""
        # 新增预处理步骤
        processed_signal = self.preprocess_signal(signal)
        rospy.loginfo(f"Processed signal: {processed_signal}")
        
        decoded_messages = []
        start_index = 0
        end_index = 9
        
        while start_index + self.signal_length <= len(processed_signal):
            message_packet = processed_signal[start_index:start_index + self.signal_length]
            end, flag = self.decode_message_packet(message_packet, end_index)
            if flag:
                decoded_messages.append(flag)
                start_index += end + 2
                end_index = end
            elif end:
                start_index += end + 2
                end_index = end
            else:
                start_index += 1
                end_index = start_index + 9
        return decoded_messages if decoded_messages else None

    # 原有聚类和拆分方法保持不变
    def cluster_samples(self, signal):
        """信号采样聚类（合并连续相同值）"""
        if not signal:
            return []
        clusters = []
        current_val = signal[0]
        count = 1
        for s in signal[1:]:
            if s == current_val:
                count += 1
            else:
                clusters.append((current_val, count))
                current_val = s
                count = 1
        clusters.append((current_val, count))
        return clusters

    def split_large_clusters(self, clusters, oversample_factor=3, tolerance=1.5):
        """处理过采样的大样本簇"""
        new_clusters = []
        for value, count in clusters:
            if count >= oversample_factor * tolerance:
                num_samples = max(1, round(count / oversample_factor))
                for _ in range(num_samples):
                    new_clusters.append((value, oversample_factor))
            else:
                new_clusters.append((value, count))
        return new_clusters


    def decode_message_packet(self, message_packet,end_index):
        """
        解码消息包。
        """
        try:
            # 1. 查找结束位
            end_index, flag2 = self.find_end_index_in_packet(message_packet,end_index,end_index+3)
            rospy.loginfo(f"End index: {end_index}, Flag2: {flag2}")

            # 2. 进行奇校验
            is_valid = self.parity_check(message_packet)
            rospy.loginfo(f"Parity check result: {is_valid}")

            if is_valid:
                # 3. 提取有效消息
                valid_message = message_packet[1:-3]  # 提取有效消息（去掉起始位和校验位）
                rospy.loginfo(f"Valid message: {valid_message}")

                # 4. 解码消息
                decoded_value = self.decode_binary_message(valid_message)
                return end_index, decoded_value
            elif flag2 == 0:
                rospy.logwarn("Parity check failed. Discarding message.")
                return end_index, None
            else:
                rospy.logwarn("End pattern not found. Discarding message.")
                return None, None
        except Exception as e:
            rospy.logerr(f"Error decoding message packet: {e}")
            return None

    def find_end_index_in_packet(self, message_packet, search_window_start=9, search_window_end=12, pattern=[1, 1, 0]):
        """
        在消息包中查找结束位。如果没有找到结束位，返回默认结束位。
        """
        flag2 = 1
        search_start = search_window_start
        search_end = min(search_window_end, len(message_packet))  # 限制最大搜索范围
        end_index = search_window_start   # 默认结束位为 Pattern indices + 8（相对于消息包）

        # 在消息包范围内查找结束位
        for i in range(search_start, search_end - 2):
            if message_packet[i:i + 3] == pattern:
                end_index = i + 2  # 结束位设置为 110 的结束位置
                flag2 = 0
                break

        return end_index, flag2

    def parity_check(self, message):
        """
        在起始位与结束位之间进行奇校验。
        """
        # 起始位后第7位为数据位
        data_start = 1
        # 校验位为结束位前一位
        parity_index = 8
        
        # 提取数据位
        data_bits = message[data_start:parity_index]
        if len(data_bits) == 0:  # 确保数据位存在
            raise ValueError("数据位长度为 0，无法进行校验。")
        
        # 计算数据位的奇校验结果
        parity_bit = message[parity_index]
        parity_calculated = 1 if data_bits.count(1) % 2 == 0 else 0  # 奇校验：求和后取模 2
        
        # 校验是否通过
        return parity_calculated == parity_bit

    def decode_binary_message(self, binary_message):
        """
        将二进制消息解码为整数值。
        """
        try:
            binary_str = ''.join(map(str, binary_message))  # 将二进制列表转换为字符串
            decoded_value = int(binary_str, 2)  # 将二进制字符串转换为整数
            rospy.loginfo(f"Pdecoded_value={decoded_value}")

            return decoded_value
        except Exception as e:
            rospy.logerr(f"Error decoding binary message: {e}")
            return None

def main():
    try:

        
        # 创建解码器节点
        decoder_node = DecoderNode(
            base_frequency=1,
            signal_length=11,
            repeat_count=3
        )
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()