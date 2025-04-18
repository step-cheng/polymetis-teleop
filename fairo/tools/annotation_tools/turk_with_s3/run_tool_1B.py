import argparse
import subprocess
import time
import sys
import os
import boto3

from create_jobs import create_turk_job
from get_results import get_hits_result

"""
Kicks off a pipeline that schedules Turk jobs for tool 1B,
collects results in batches and collates data.

1. Read in newline separated commands and construct CSV input.
2. Create HITs for each input command using tool 1B template.
3. Continuously check for completed assignments, fetching results in batches.
4. Collate turk output data with the input job specs.
5. Postprocess datasets to obtain well formed action dictionaries.
"""

parser = argparse.ArgumentParser()
parser.add_argument("--default_write_dir", type=str, required=True)
parser.add_argument("--dev", default=False, action="store_true")
parser.add_argument("--timeout", type=float, default=60)

args = parser.parse_args()
dev_flag = "--dev" if args.dev else ""
default_write_dir = args.default_write_dir
timeout = args.timeout

# CSV input
rc = subprocess.call(
    [
        f"python3 construct_input_for_turk.py --input_file {default_write_dir}/B/input.txt --tool_num 2 > {default_write_dir}/B/turk_input.csv"
    ],
    shell=True,
)
if rc != 0:
    print("Error preprocessing. Exiting.")
    sys.exit()

hit_id = create_turk_job("fetch_question_B.xml", 2, f"{default_write_dir}/B/turk_input.csv", f"{default_write_dir}/B/turk_job_specs.csv", dev_flag)

# Wait for results to be ready
print("Turk jobs created for tool B at : %s \n Waiting for results..." % time.ctime())
print("*"*50)

access_key = os.getenv("MTURK_AWS_ACCESS_KEY_ID")
secret_key = os.getenv("MTURK_AWS_SECRET_ACCESS_KEY")
aws_region = os.getenv("MTURK_AWS_REGION", default="us-east-1")

if dev_flag:
    MTURK_URL = "https://mturk-requester-sandbox.{}.amazonaws.com".format(aws_region)
else:
    MTURK_URL = "https://mturk-requester.{}.amazonaws.com".format(aws_region)

mturk = boto3.client(
    "mturk",
    aws_access_key_id=access_key,
    aws_secret_access_key=secret_key,
    region_name=aws_region,
    endpoint_url=MTURK_URL,
)

get_hits_result(mturk, hit_id, f"{default_write_dir}/B/turk_output.csv", True if dev_flag else False, timeout)

# Collate datasets
print("*"*50)
print("*** Collating turk outputs and input job specs ***")
rc = subprocess.call([f"python3 collate_answers.py --turk_output_csv {default_write_dir}/B/turk_output.csv --job_spec_csv {default_write_dir}/B/turk_job_specs.csv --collate_output_csv {default_write_dir}/B/processed_outputs.csv"], shell=True)
if rc != 0:
    print("Error collating answers. Exiting.")
    sys.exit()


# Postprocess
print("*"*50)
print("*** Postprocessing results ***")
rc = subprocess.call([f"python3 parse_tool_B_outputs.py --folder_name {default_write_dir}/B/"], shell=True)
if rc != 0:
    print("Error postprocessing tool B. Exiting.")
print("*"*50)