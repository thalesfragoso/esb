#! /usr/bin/env python3

import subprocess
import sys


def run(args):
    print("Running `{}`...".format(" ".join(args)))
    ret = subprocess.call(args) == 0
    print("")
    return ret


def main():

    cargo_cmd = ["cargo", "build"]

    features = [["--features=51"],
                ["--features=52810"],
                ["--features=52840"]]

    targets = [["--target=thumbv6m-none-eabi"],
               ["--target=thumbv7em-none-eabi"],
               ["--target=thumbv7em-none-eabihf"]]

    if not all(map(lambda f, t: run(cargo_cmd + f + t),
                   features, targets)):
        sys.exit(-1)


if __name__ == "__main__":
    main()
