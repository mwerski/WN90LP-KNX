#!/usr/bin/env python3
"""Klont die OpenKNX-Abhaengigkeiten aus dependencies.txt nach lib/ und
checkt den dort festgehaltenen Stand aus. Mehrfach ausfuehrbar.
Aufruf im Projektverzeichnis: python3 restore/restore.py"""
import pathlib
import subprocess
import sys

# Projektverzeichnis = Ordner mit dependencies.txt (Aufrufort, Skriptordner oder dessen Elternordner)
skript = pathlib.Path(__file__).resolve().parent
kandidaten = [pathlib.Path.cwd(), skript, skript.parent]
root = next((p for p in kandidaten if (p / "dependencies.txt").is_file()), None)
if root is None:
    sys.exit("dependencies.txt nicht gefunden. Bitte im Projektverzeichnis aufrufen.")
print(f"Projektverzeichnis: {root}")


def git(*args, cwd=None):
    print("  git " + " ".join(args))
    subprocess.run(["git", *args], cwd=cwd, check=True)


for line in (root / "dependencies.txt").read_text(encoding="utf-8").splitlines():
    if not line.strip() or line.startswith("-") or line.startswith("#"):
        continue
    commit, branch, folder, url = line.split(maxsplit=3)
    target = root / folder
    print(f"{folder} ({branch} @ {commit})")
    if (target / ".git").exists():
        git("fetch", "origin", branch, cwd=target)
    else:
        target.parent.mkdir(parents=True, exist_ok=True)
        git("clone", "--branch", branch, url, str(target))
    git("checkout", "--quiet", commit, cwd=target)

print("Fertig.")
