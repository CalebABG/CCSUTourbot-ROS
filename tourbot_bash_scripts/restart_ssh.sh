#!/usr/bin/env bash

sudo systemctl stop ssh
sudo systemctl start ssh
sudo systemctl reload ssh
sudo systemctl status ssh