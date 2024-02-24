import rclpy
from rclpy.node import Node

# メイン
def main():
    # ROS通信の初期化
    rclpy.init()
    
    # ノードの生成   
    node = Node("hello_node")
    
    # ログ出力
    print("Hello World!")

    # ノード終了の待機
    rclpy.spin(node)
    
    # ROS通信のシャットダウン
    rclpy.shutdown()

if __name__ == "__main__":
   main()
