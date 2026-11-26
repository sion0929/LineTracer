#!/usr/bin/env python3
"""Drive the oracle route while logging the follower's truth-aligned perception map."""
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main():
    # Parse the range override before importing vision, which reads it on import.
    if sys.argv[1:2] == ["--xfar"]:
        os.environ["LF_X_FAR"] = sys.argv[2]
        del sys.argv[1:3]

    from run_oracle import OracleNode, PKG, main as run_oracle

    sys.path.insert(0, str(HERE.parent / "user_base"))
    from map_logger import MapLogger

    class OracleMapNode(MapLogger, OracleNode):
        save_empty_map = True

    os.environ.setdefault("LF_PARAMS", str(PKG / "config/rule.yaml"))
    run_oracle(OracleMapNode)


if __name__ == "__main__":  # Truth's spawn child re-imports this file.
    main()
