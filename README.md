# WATonomous ASD Admissions Assignment

##Demo Video
https://youtu.be/k1j8tp_qUvk

## Implemented navigation

This implementation adds lidar costmaps, global map memory, A* planning, and
pure pursuit control in C++ ROS 2. See [NAVIGATION.md](NAVIGATION.md) for the
architecture, Windows launcher commands, regression tests, and verified simulator
results. Run `powershell -ExecutionPolicy Bypass -File .\run.ps1 Build` to build
the Docker images and execute the core tests.

The starter repository and simulator are from WATonomous. Navigation code,
diagnostics, and tests were developed with ChatGPT/Codex assistance and tested
through user-run Docker builds and simulator traces.

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/
