#!/bin/bash

git config --global --add core.pager "less -F -X"
git config --global --add safe.directory /home/ee/sensing_fw

west sdk install