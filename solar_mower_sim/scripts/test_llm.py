#!/usr/bin/env python3
"""ROS なしで LLM の変換精度を確認するスクリプト (動画の冒頭シーン用にも使える)。

usage: python3 scripts/test_llm.py [model]
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from solar_mower_sim.llm_commander import OllamaClient, validate  # noqa: E402

model = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("OLLAMA_MODEL", "qwen2.5:7b")
prompt = open(os.path.join(os.path.dirname(__file__), "..", "prompts", "system_prompt.txt"),
              encoding="utf-8").read()
llm = OllamaClient(model, os.environ.get("OLLAMA_HOST", "http://localhost:11434"), prompt)

TESTS = [
    "パネルに沿って10メートル進んで",
    "右側の架台から60センチ離れてゆっくり走って",
    "北を向いて8m真っ直ぐ進め",
    "パネル沿いに12m刈ったら、右に90度曲がって5m進んで止まって",
    "Uターンして",
    "止まれ",
    "今日の天気は？",
    "Follow the panel row on your left for 6 meters at 0.8 m/s",
]

for t in TESTS:
    t0 = time.time()
    try:
        cmd = llm.to_command(t)
        err = validate(cmd)
    except Exception as e:  # noqa: BLE001
        cmd, err = None, str(e)
    print(f"\n指示: {t}\n  → {json.dumps(cmd, ensure_ascii=False)}  "
          f"[{time.time()-t0:.1f}s] {'OK' if err is None else 'NG: ' + err}")
