"""Test what a single Roboflow inference call returns, before batch-labeling
all images. Run this on the laptop where data/raw_frames/ lives.
Set ROBOFLOW_API_KEY as an environment variable first - don't hardcode it.
"""
import os
import json
from inference_sdk import InferenceHTTPClient

client = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key=os.environ["ROBOFLOW_API_KEY"],
)

IMAGES_DIR = "data/raw_frames"      # relative to the repo root
test_image = sorted(os.listdir(IMAGES_DIR))[0]
test_path = os.path.join(IMAGES_DIR, test_image)

result = client.run_workflow(
    workspace_name="chuoiuhuhu",
    workflow_id="find-yellow-duck-toy",
    images={"image": test_path},
    use_cache=True,
)

print(f"Tested on: {test_path}\n")
print(json.dumps(result, indent=2))
