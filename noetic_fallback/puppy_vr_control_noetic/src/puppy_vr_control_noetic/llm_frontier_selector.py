#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""llama.cpp가 frontier ID만 선택하도록 제한하는 고수준 판단기."""

import json
import urllib.request


def parse_frontier_choice(text, allowed_ids):
    """모델 문자열에서 허용된 frontier_id만 반환."""
    start = text.find('{')
    end = text.rfind('}')
    if start < 0 or end <= start:
        return None
    try:
        value = json.loads(text[start:end + 1])
    except (TypeError, ValueError):
        return None
    frontier_id = value.get('frontier_id') if isinstance(value, dict) else None
    if isinstance(frontier_id, bool) or not isinstance(frontier_id, int):
        return None
    return frontier_id if frontier_id in set(allowed_ids) else None


class LlmFrontierSelector(object):
    """Qwen은 속도나 시간을 만들지 않고 제공된 후보 ID만 고른다."""

    def __init__(self, url='http://127.0.0.1:8081/completion', timeout_sec=8.0):
        self.url = url
        self.timeout_sec = float(timeout_sec)
        self.last_raw = ''

    def choose(self, candidates, recent_goals=None):
        if not candidates:
            return None
        compact = []
        for item in candidates:
            compact.append({
                'frontier_id': item['id'],
                'distance_m': round(item['distance_m'], 2),
                'bearing_deg': round(item.get('bearing_deg', 0.0), 1),
                'unexplored_size': item['cluster_size'],
            })
        prompt = (
            '너는 미로를 탐색하는 사족보행 로봇의 고수준 판단기다.\n'
            '가깝고 미탐사 영역이 큰 후보를 우선하고 같은 위치의 반복 선택을 피한다.\n'
            '반드시 제공된 frontier_id 하나만 선택한다. 속도와 시간은 결정하지 않는다.\n'
            '후보: %s\n최근 선택: %s\n'
            'JSON만 출력: {"frontier_id": 정수}\n' %
            (json.dumps(compact, ensure_ascii=False),
             json.dumps(list(recent_goals or ())[-5:], ensure_ascii=False)))
        payload = {
            'prompt': prompt,
            'n_predict': 48,
            'temperature': 0.0,
            'stop': ['\n\n', '\n사용자:'],
        }
        request = urllib.request.Request(
            self.url,
            data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
            result = json.loads(response.read().decode('utf-8'))
        self.last_raw = result.get('content', '').strip()
        return parse_frontier_choice(
            self.last_raw, [candidate['id'] for candidate in candidates])
