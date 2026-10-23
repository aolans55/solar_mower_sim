#!/usr/bin/env python3
"""/llm_log (LLM の入力と出力) と /mission_status を見やすく表示する。

デモ動画で「日本語の指示 → LLM が生成した JSON → ロボットの実行状況」を並べて見せる用。
usage (シミュレーション起動中に): ./sim python3 /ws/src/solar_mower_sim/scripts/show_llm_log.py
"""
import json
import time

import rclpy
from std_msgs.msg import String

CYAN, YELLOW, GREEN, RED, DIM, RESET = "\033[36m", "\033[33m", "\033[32m", "\033[31m", "\033[2m", "\033[0m"


def on_llm(msg: String) -> None:
    try:
        log = json.loads(msg.data)
    except json.JSONDecodeError:
        print(msg.data, flush=True)
        return
    print(f"\n{CYAN}━━ /llm_log ━━{RESET}  {time.strftime('%H:%M:%S')}")
    print(f"{YELLOW}指示 (入力){RESET}: {log.get('input')}")
    print(f"{GREEN}LLM が生成したコマンド (出力){RESET}:")
    print(json.dumps(log.get("command"), ensure_ascii=False, indent=2))
    if log.get("error"):
        print(f"{RED}検証エラー → 実行しない{RESET}: {log['error']}")
    else:
        print(f"{GREEN}スキーマ検証 OK → /mission_cmd へ{RESET}")
    print(flush=True)


def on_status(msg: String) -> None:
    text = msg.data
    if len(text) > 70:
        text = text[:67] + "..."
    print(f"{DIM}[/mission_status]{RESET} {text}", flush=True)


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("show_llm_log")
    node.create_subscription(String, "llm_log", on_llm, 10)
    node.create_subscription(String, "mission_status", on_status, 10)
    print(f"{CYAN}LLM ログ / ミッション状況を表示中 ...{RESET}", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
