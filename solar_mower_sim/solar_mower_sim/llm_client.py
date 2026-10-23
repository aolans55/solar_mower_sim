"""ROS 非依存の LLM クライアントとコマンド検証 (ROS なしで test_llm.py から使える)。

LLM サーバーは OpenAI 互換の /v1/chat/completions を話すものなら何でもよい。
  - LM Studio (Windows, 既定) : LLM_BASE_URL=http://localhost:1234/v1
  - Ollama (DGX Spark 等)     : LLM_BASE_URL=http://<host>:11434/v1  LLM_MODEL=qwen2.5:7b
"""
import json
import os
import re
from typing import Any, Dict, Optional

import requests

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


def default_base_url() -> str:
    """LLM_BASE_URL > OLLAMA_HOST (+/v1, 旧設定との互換) > LM Studio 既定"""
    if os.environ.get("LLM_BASE_URL"):
        return os.environ["LLM_BASE_URL"]
    if os.environ.get("OLLAMA_HOST"):
        return os.environ["OLLAMA_HOST"].rstrip("/") + "/v1"
    return "http://localhost:1234/v1"


def extract_json(content: str) -> Dict[str, Any]:
    """応答テキストから JSON オブジェクトを取り出す (<think> ブロックやコードブロックを除去)"""
    content = re.sub(r"<think>.*?</think>", "", content, flags=re.DOTALL).strip()
    i, j = content.find("{"), content.rfind("}")
    if i < 0 or j < i:
        raise ValueError(f"no JSON object in response: {content[:200]!r}")
    return json.loads(content[i:j + 1])


class LLMClient:
    """OpenAI 互換 Chat Completions API クライアント (LM Studio / Ollama / vLLM 等)"""

    def __init__(self, base_url: str, model: str, system_prompt: str,
                 api_key: str = "", reasoning_effort: str = "none", timeout: float = 60.0):
        self.base_url = base_url.rstrip("/")
        self.system_prompt = system_prompt
        self.reasoning_effort = reasoning_effort
        self.timeout = timeout
        self.headers = {"Authorization": f"Bearer {api_key or 'not-needed'}"}
        self.model = model or self.first_model()

    def first_model(self) -> str:
        """モデル未指定ならサーバーの /models から最初のチャットモデルを選ぶ"""
        r = requests.get(self.base_url + "/models", headers=self.headers, timeout=self.timeout)
        r.raise_for_status()
        ids = [m["id"] for m in r.json().get("data", []) if "embed" not in m["id"]]
        if not ids:
            raise RuntimeError(f"no chat model available at {self.base_url}")
        return ids[0]

    def to_command(self, text: str) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "model": self.model,
            "temperature": 0.0,
            "stream": False,
            "messages": [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": text},
            ],
        }
        # 思考 (thinking) モデルの推論を止めて応答を速くする (LM Studio で 45s → 5s)
        if self.reasoning_effort:
            payload["reasoning_effort"] = self.reasoning_effort
        url = self.base_url + "/chat/completions"
        r = requests.post(url, json=payload, headers=self.headers, timeout=self.timeout)
        if r.status_code == 400 and "reasoning_effort" in payload:
            # reasoning_effort 非対応のサーバー向けに外して再送
            payload.pop("reasoning_effort")
            r = requests.post(url, json=payload, headers=self.headers, timeout=self.timeout)
        r.raise_for_status()
        return extract_json(r.json()["choices"][0]["message"]["content"] or "")
