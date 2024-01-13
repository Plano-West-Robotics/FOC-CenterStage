import subprocess
import re
import sys

ignored_paths = [
    b"FtcRobotController",
]

authors = [
    "missing",
    "Yash Karandikar",
    "gallant",
]

output = subprocess.run([
    "git",
    "rev-list",
    *map(lambda a : f"--author={a}", authors),
    "--author-date-order",
    "--reverse",
    "master",
], capture_output=True).stdout
rev_list = output.decode("utf-8").split("\n")

gource_input = b""
for i, hash in enumerate(rev_list):
    print(f"processing {i}/{len(rev_list)} commits...\r", end="")

    if hash == "":
        continue
    
    output = subprocess.run([
        "git",
        "cat-file",
        "commit",
        hash
    ], capture_output=True).stdout
    author_match = re.search(b"author (.+) <.*> ([0-9]+) [-+][0-9]{4}", output)
    author = author_match.group(1)
    timestamp = author_match.group(2)

    output = subprocess.run([
        "git",
        "diff",
        hash,
        f"{hash}^",
        "--name-status",
        "--no-renames",
        "-z",
    ], capture_output=True).stdout
    while len(output) > 0:
        op_kind = output[0:1]
        output = output[1:]
        assert op_kind in [b"M", b"A", b"D"]
        
        assert output[0:1] == b"\0"
        output = output[1:]

        filename, sep, output = output.partition(b"\0")
        assert sep == b"\0"

        if filename.startswith(tuple(ignored_paths)):
            continue

        gource_input += timestamp
        gource_input += b"|"
        gource_input += author
        gource_input += b"|"
        gource_input += op_kind
        gource_input += b"|"
        gource_input += filename
        gource_input += b"\n"

print(f"processing {len(rev_list)}/{len(rev_list)} commits...")

gource_cmd = ["gource", "--log-format", "custom", "-"] + sys.argv[1:]
print("launching gource...")
print("> " + " ".join(gource_cmd))
subprocess.run(gource_cmd, input=gource_input)