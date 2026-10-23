#!/usr/bin/env python3
"""自然言語 → JSON ミッションコマンド変換ノード (Ollama ローカル LLM)。

入力: 端末の標準入力 (対話) または /voice_text (std_msgs/String)
出力: /mission_cmd (std_msgs/String, JSON)

LLM は「指示の解釈」だけを担当し、生成した JSON は executor 側で
スキーマ検証されるので、LLM の誤出力がそのまま駆動系に届くことはない。
"""
import json
import os
import sys
import threading
from typing import Any, Dict, Optional

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

VALID_ACTIONS = {"follow_panel", "go_straight", "turn", "stop", "sequence"}


def validate(cmd: Any) -> Optional[str]:
    """スキーマ検証。問題があればエラーメッセージ、OK なら None"""
    if not isinstance(cmd, dict):
        return "not an object"
    a = cmd.get("action")
    if a not in VALID_ACTIONS:
        return f"unknown action: {a}"
    if a == "sequence":
        steps = cmd.get("steps")
        if not isinstance(steps, list) or not steps:
            return "sequence needs non-empty steps"
        for s in steps:
            if (e := validate(s)) is not None:
                return e
            if s.get("action") == "sequence":
                return "nested sequence not allowed"
    # 数値の安全範囲にクランプ
    if "speed" in cmd:
        cmd["speed"] = max(0.1, min(1.0, float(cmd["speed"])))
    if "distance" in cmd and a == "follow_panel":
        cmd["distance"] = max(0.4, min(2.0, float(cmd["distance"])))
    if "distance" in cmd and a == "go_straight":
        cmd["distance"] = max(0.0, min(50.0, float(cmd["distance"])))
    if "length" in cmd:
        cmd["length"] = max(0.0, min(50.0, float(cmd["length"])))
    if "angle_deg" in cmd:
        cmd["angle_deg"] = max(-180.0, min(180.0, float(cmd["angle_deg"])))
    return None


class OllamaClient:
    def __init__(self, model: str, host: str, system_prompt: str, timeout: float = 60.0):
        self.model = model
        self.url = host.rstrip("/") + "/api/chat"
        self.system_prompt = system_prompt
        self.timeout = timeout

    def to_command(self, text: str) -> Dict[str, Any]:
        payload = {
            "model": self.model,
            "stream": False,
            "format": "json",
            "options": {"temperature": 0.0},
            "messages": [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": text},
            ],
        }
        r = requests.post(self.url, json=payload, timeout=self.timeout)
        r.raise_for_status()
        content = r.json()["message"]["content"].strip()
        # 念のためコードブロック除去
        if content.startswith("```"):
            content = content.strip("`")
            content = content[content.find("{"):content.rfind("}") + 1]
        return json.loads(content)


class LLMCommander(Node):
    def __init__(self) -> None:
        super().__init__("llm_commander")
        self.declare_parameter("model", os.environ.get("OLLAMA_MODEL", "qwen2.5:7b"))
        self.declare_parameter("host", os.environ.get("OLLAMA_HOST", "http://localhost:11434"))
        self.declare_parameter("prompt_file", "")
        self.declare_parameter("interactive", True)

        prompt_file = self.get_parameter("prompt_file").value
        if not prompt_file:
            from ament_index_python.packages import get_package_share_directory
            prompt_file = os.path.join(get_package_share_directory("solar_mower_sim"),
                                       "prompts", "system_prompt.txt")
        with open(prompt_file, encoding="utf-8") as f:
            system_prompt = f.read()

        self.llm = OllamaClient(self.get_parameter("model").value,
                                self.get_parameter("host").value, system_prompt)
        self.pub = self.create_publisher(String, "mission_cmd", 10)
        self.pub_log = self.create_publisher(String, "llm_log", 10)
        self.create_subscription(String, "voice_text", self.on_text, 10)
        self.get_logger().info(f"llm_commander ready (model={self.llm.model})")

        if self.get_parameter("interactive").value:
            threading.Thread(target=self.stdin_loop, daemon=True).start()

    def on_text(self, msg: String) -> None:
        self.handle(msg.data)

    def stdin_loop(self) -> None:
        print("\n==== 自然言語で指示を入力してください (Ctrl-D で終了) ====", flush=True)
        for line in sys.stdin:
            line = line.strip()
            if line:
                self.handle(line)
            print("> ", end="", flush=True)

    def handle(self, text: str) -> None:
        self.get_logger().info(f'指示: "{text}"')
        try:
            cmd = self.llm.to_command(text)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"LLM error: {e}")
            return
        err = validate(cmd)
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
