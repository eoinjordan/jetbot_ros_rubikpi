#!/usr/bin/env python3
import argparse
import json
import hmac
import hashlib
import os
import time
import uuid
from typing import List

import requests


def build_payload(values: List[List[float]], interval_ms: int, sensor_names: List[str], sensor_units: List[str]) -> dict:
    empty_signature = '0' * 64
    device_name = ":".join([f"{uuid.getnode():012x}"[i:i + 2] for i in range(0, 12, 2)])

    sensors = [{"name": name, "units": unit} for name, unit in zip(sensor_names, sensor_units)]

    return {
        "protected": {
            "ver": "v1",
            "alg": "HS256",
            "iat": time.time(),
        },
        "signature": empty_signature,
        "payload": {
            "device_name": device_name,
            "device_type": "RUBIK_PI",
            "interval_ms": interval_ms,
            "sensors": sensors,
            "values": values,
        },
    }


def sign_payload(payload: dict, hmac_key: str) -> str:
    encoded = json.dumps(payload)
    signature = hmac.new(hmac_key.encode('utf-8'), msg=encoded.encode('utf-8'), digestmod=hashlib.sha256).hexdigest()
    payload['signature'] = signature
    return json.dumps(payload)


def load_csv(path: str) -> List[List[float]]:
    rows: List[List[float]] = []
    with open(path, 'r', encoding='utf-8') as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            rows.append([float(x) for x in line.split(',')])
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description='Upload custom motor health data to Edge Impulse')
    parser.add_argument('--input', required=True, help='CSV file containing rows of sensor values')
    parser.add_argument('--interval-ms', type=int, default=16, help='Sampling interval in ms')
    parser.add_argument('--sensor-names', default='vibration_x,vibration_y,vibration_z', help='Comma-separated sensor names')
    parser.add_argument('--sensor-units', default='g,g,g', help='Comma-separated sensor units')
    parser.add_argument('--api-key', default=os.environ.get('EDGE_IMPULSE_API_KEY', ''), help='Edge Impulse API key')
    parser.add_argument('--hmac-key', default=os.environ.get('EDGE_IMPULSE_HMAC_KEY', ''), help='Edge Impulse HMAC key')
    parser.add_argument('--label', default='motor_health', help='Label for the uploaded sample')

    args = parser.parse_args()

    if not args.api_key or not args.hmac_key:
        raise SystemExit('Missing API key or HMAC key. Provide via --api-key/--hmac-key or env vars.')

    values = load_csv(args.input)
    sensor_names = [s.strip() for s in args.sensor_names.split(',')]
    sensor_units = [s.strip() for s in args.sensor_units.split(',')]

    if len(sensor_names) != len(sensor_units):
        raise SystemExit('sensor-names and sensor-units must have the same length')

    payload = build_payload(values, args.interval_ms, sensor_names, sensor_units)
    signed = sign_payload(payload, args.hmac_key)

    res = requests.post(
        url='https://ingestion.edgeimpulse.com/api/training/data',
        data=signed,
        headers={
            'Content-Type': 'application/json',
            'x-file-name': args.label,
            'x-api-key': args.api_key,
        },
        timeout=30,
    )

    if res.status_code == 200:
        print('Uploaded file to Edge Impulse', res.status_code)
    else:
        print('Failed to upload file', res.status_code, res.text)


if __name__ == '__main__':
    main()
