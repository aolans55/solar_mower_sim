#!/usr/bin/env python3
"""自然言語 → JSON ミッションコマンド変換ノード (OpenAI 互換 API のローカル LLM)。

入力: 端末の標準入力 (対話) または /voice_text (std_msgs/String)
出力: /mission_cmd (std_msgs/String, JSON)

LLM 呼び出しと検証は llm_client.py (ROS 非依存)。接続先は LLM_BASE_URL 等で切り替える。

LLM は「指示の解釈」だけを担当し、生成した JSON は validate() で
スキーマ検証されるので、LLM の誤出力がそのまま駆動系に届くことはない。
"""
import json
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .llm_client import LLMClient, default_base_url, validate


class LLMCommander(Node):
    def __init__(self) -> None:
        super().__init__("llm_commander")
        self.declare_parameter("base_url", default_base_url())
        self.declare_parameter("model", os.environ.get("LLM_MODEL", os.environ.get("OLLAMA_MODEL", "")))
        self.declare_parameter("api_key", os.environ.get("LLM_API_KEY", ""))
        self.declare_parameter("reasoning_effort", os.environ.get("LLM_REASONING_EFFORT", "none"))
        self.declare_parameter("prompt_file", "")
        self.declare_parameter("interactive", True)

        prompt_file = self.get_parameter("prompt_file").value
        if not prompt_file:
            from ament_index_python.packages import get_package_share_directory
            prompt_file = os.path.join(get_package_share_directory("solar_mower_sim"),
                                       "prompts", "system_prompt.txt")
        with open(prompt_file, encoding="utf-8") as f:
            system_prompt = f.read()

        self.llm = LLMClient(self.get_parameter("base_url").value,
                             self.get_parameter("model").value, system_prompt,
                             api_key=self.get_parameter("api_key").value,
                             reasoning_effort=self.get_parameter("reasoning_effort").value)
        self.pub = self.create_publisher(String, "mission_cmd", 10)
        self.pub_log = self.create_publisher(String, "llm_log", 10)
        self.create_subscription(String, "voice_text", self.on_text, 10)
        self.get_logger().info(f"llm_commander ready (model={self.llm.model}, url={self.llm.base_url})")

        if self.get_parameter("interactive").value:
            threading.Thread(target=self.stdin_loop, daemon=True).start()

    def on_text(self, msg: String) -> None:
        self.handle_text(msg.data)

    def stdin_loop(self) -> None:
        print("\n==== 自然言語で指示を入力してください (Ctrl-D で終了) ====", flush=True)
        print("> ", end="", flush=True)
        for line in sys.stdin:
            line = line.strip()
            if line:
                self.handle_text(line)
            print("> ", end="", flush=True)

    def handle_text(self, text: str) -> None:
        self.get_logger().info(f'指示: "{text}"')
        try:
            cmd = self.llm.to_command(text)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"LLM error: {e}")
            return
        try:
            err = validate(cmd)
        except (TypeError, ValueError) as e:  # 数値でない値 ("speed": "slow" 等)
            err = f"invalid value: {e}"
        log = {"input": text, "command": cmd, "error": err}
        self.pub_log.publish(String(data=json.dumps(log, ensure_ascii=False)))
        if err:
            self.get_logger().error(f"rejected command ({err}): {cmd}")
            return
        js = json.dumps(cmd, ensure_ascii=False)
        self.get_logger().info(f"→ {js}")
        self.pub.publish(String(data=js))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LLMCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
