#!/bin/python

import os
import sys
import subprocess
from dataclasses import dataclass
import shutil


@dataclass
class Artifact:
    path: str
    readable_name: str

def artifact_from_string(input: str) -> Artifact:
    inputs = input.split(":")
    return Artifact(inputs[0], inputs[1])


def prepare_files_for_artifact_upload(version_info: str, output_dir: str, artifacts: list[Artifact]):
    """
    Copies the files in the "artifacts" array, renames them to include the short git hash
    and version number.

    Arguments:
    version_info: str - Version information
    output_dir - Output directory for the artifacts
    artifacts - In "full_path:final_name" form
    
    Return:
    No return
    """

    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)

    os.makedirs(output_dir)

    for artifact in artifacts:
        filename, extension = os.path.splitext(artifact.path)
        new_path = f"{output_dir}/{artifact.readable_name}-{version_info}{extension}"
        shutil.copy(artifact.path, new_path)


if __name__ == "__main__":
    un_parsed_artifacts = sys.argv[3:]
    artifacts = [artifact_from_string(x) for x in un_parsed_artifacts]

    prepare_files_for_artifact_upload(sys.argv[1], sys.argv[2], artifacts)
