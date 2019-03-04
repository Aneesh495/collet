#!/usr/bin/env python3
import os
import sys

repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
target_codepoint = chr(0x2014)
found = False

for root, dirs, files in os.walk(repo_root):
    if '.git' in dirs:
        dirs.remove('.git')
    if 'build' in dirs:
        dirs.remove('build')
    for f in files:
        if f.endswith(('.gleam', '.md', '.toml', '.json', '.sh', '.py', '.c', '.h', '.txt')):
            path = os.path.join(root, f)
            with open(path, 'r', encoding='utf-8', errors='ignore') as fp:
                for line_no, line in enumerate(fp, start=1):
                    if target_codepoint in line:
                        print(f"ERROR: Em dash found in {os.path.relpath(path, repo_root)}:{line_no}: {line.strip()}")
                        found = True

if found:
    sys.exit(1)
else:
    print("Verification passed: No em dashes found.")
    sys.exit(0)
