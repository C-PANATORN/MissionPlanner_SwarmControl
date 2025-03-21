# MissionPlanner_SwarmControl

- [Introduction](#introduction)
  - [Getting Started](#getting-started)
  - [Installation](#installation)
  - [GitHub Action](#github-action)
- [Usage](#usage)
  - [Options](#options)
    - [Read from stdin](#read-from-stdin)
    - [Add a preface](#add-a-preface)
    - [Offset heading levels](#offset-heading-levels)
    - [Disable the TOC](#disable-the-toc)
    - [Write to file](#write-to-file)
    - [Change the directory](#change-the-directory)
    - [Report a diff](#report-a-diff)
  - [Syntax](#syntax)
- [Advanced](#advanced)
  - [Page Titles](#page-titles)
  - [Absorbing headings](#absorbing-headings)
  - [Including summaries](#including-summary-files)
- [License](#license)

## Introduction

<div align="center">

<img src="doc/images/stitchmd-logo.png" width="300"/>

[![CI](https://github.com/abhinav/stitchmd/actions/workflows/ci.yml/badge.svg)](https://github.com/abhinav/stitchmd/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/abhinav/stitchmd/branch/main/graph/badge.svg?token=MBOK2PHS0X)](https://codecov.io/gh/abhinav/stitchmd)

</div>

stitchmd is a tool that stitches together several Markdown files
into one large Markdown file,
making it easier to maintain larger Markdown files.

It lets you define the layout of your final document in a **summary file**,
which it then uses to stitch and interlink other Markdown files with.

<div align="center">

![Flow diagram](doc/images/flow.png)

</div>

See [Getting Started](#getting-started) for a tutorial,
or [Usage](#usage) to start using it.

### Features

- **Cross-linking**:
  Recognizes cross-links between files and their headers
  and re-targets them for their new locations.
  This keeps your input and output files
  independently browsable on websites like GitHub.

    <details>
    <summary>Example</summary>

  **Input**

  ```markdown
  [Install](install.md) the program.
  See also, [Overview](#overview).
  ```

  **Output**

  ```markdown
  [Install](#install) the program.
  See also, [Overview](#overview).
  ```

    </details>

- **Relative linking**:
  Rewrites relative images and links to match their new location.

    <details>
    <summary>Example</summary>

  **Input**

  ```markdown
  ![Graph](images/graph.png)
  ```

  **Output**

  ```markdown
  ![Graph](docs/images/graph.png)
  ```

    </details>

- **Header offsetting**:
  Adjusts levels of all headings in included Markdown files
  based on the hierarchy in the summary file.

    <details>
    <summary>Example</summary>

  **Input**

  ```markdown
  - [Introduction](intro.md)
    - [Installation](install.md)
  ```

  **Output**

  ```markdown
  # Introduction

  <!-- contents of intro.md -->

  ## Installation

  <!-- contents of install.md -->
  ```

    </details>

### Use cases

The following is a non-exhaustive list of use cases
where stitchmd may come in handy.

- Maintaining a document with several collaborators
  with reduced risk of merge conflicts.
- Divvying up a document between collaborators by ownership areas.
  Owners will work inside the documents or directories assigned to them.
- Keeping a single-page and multi-page version of the same content.
- Re-using documentation across multiple Markdown documents.
- Preparing initial drafts of long-form content
  from an outline of smaller texts.

...and more.
(Feel free to contribute a PR with your use case.)

### Getting Started

This is a step-by-step tutorial to introduce stitchmd.

For details on how to use it, see [Usage](#usage).

1. First, [install stitchmd](#installation).
   If you have Go installed, this is as simple as:

   ```bash
   go install go.abhg.dev/stitchmd@latest
   ```

   For other installation methods, see the [Installation](#installation) section.

2. Create a couple Markdown files.
   Feel free to open these up and add content to them.

   ```bash
   echo 'Welcome to my program.' > intro.md
   echo 'It has many features.' > features.md
   echo 'Download it from GitHub.' > install.md
   ```

   Alternatively, clone this repository and copy the [doc folder](doc).

3. Create a summary file defining the layout between these files.

   ```bash
   cat > summary.md << EOF
   - [Introduction](intro.md)
     - [Features](features.md)
   - [Installation](install.md)
   EOF
   ```

4. Run stitchmd on the summary.

   ```bash
   stitchmd summary.md
   ```

   The output should look similar to the following:

   ```markdown
   - [Introduction](#introduction)
     - [Features](#features)
   - [Installation](#installation)

   # Introduction

   Welcome to my program.

   ## Features

   It has many features.

   # Installation

   Download it from GitHub.
   ```

   Each included document got its own heading
   matching its level in the summary file.

5. Next, open up `intro.md` and add the following to the bottom:

   ```markdown
   See [installation](install.md) for instructions.
   ```

   If you run stitchmd now, the output should change slightly.

   ```markdown
   - [Introduction](#introduction)
     - [Features](#features)
   - [Installation](#installation)

   # Introduction

   Welcome to my program.
   See [installation](#installation) for instructions.

   ## Features

   It has many features.

   # Installation

   Download it from GitHub.
   ```

   stitchmd recognized the link from `intro.md` to `install.md`,
   and updated it to point to the `# Installation` header instead.

**Next steps**: Play around with the document further:

- Alter the hierarchy further.
- Add an item to the list without a file:

  ```markdown
  - Overview
    - [Introduction](intro.md)
    - [Features](features.md)
  ```

- Add sections or subsections to a document and link to those.

  ```markdown
  [Build from source](install.md#build-from-source).
  ```

- Add a heading to the `summary.md`:

  ```markdown
  # my awesome program

  - [Introduction](#introduction)
    - [Features](#features)
  - [Installation](#installation)
  ```

### Installation

This repository uses the Gazebo11 simulator engine with Ardupilot STIL via MAVROS communication protocol.

Please refer to the following repositories for further details.
- [Intelligent-Quads/iq_sim](https://github.com/Intelligent-Quads/iq_sim)  
- [monemati/multiuav-gazebo-simulation](https://github.com/monemati/multiuav-gazebo-simulation) 

#### Ardupilot installation

This repository uses the latest verion of Ardupilot as of time of writting (Arducopter V4.5.7 and Copter-4.5.7)

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install git

git clone --recurse-submodules https://github.com/your-github-userid/ardupilot
git clone https://github.com/ArduPilot/ardupilot.git

ls
cd ardupilot/
Tools/environment_install/install-prereqs-ubuntu.sh -y
mavproxy.py --version
pip show MAVProxy
. ~/.profile


git checkout Copter-4.5.7
git submodule update --init --recursive
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

##### Gazebo installation

- [monemati/multiuav-gazebo-simulation](https://github.com/osrf/gazebo_tutorials/blob/master/install_ubuntu/tutorial_11.0.md)

If you use **Homebrew** on macOS or Linux,
run the following command to install stitchmd:

```bash
brew install abhinav/tap/stitchmd
```

##### ArchLinux

If you use **ArchLinux**,
install stitchmd from [AUR](https://aur.archlinux.org/)
using the [stitchmd-bin](https://aur.archlinux.org/packages/stitchmd-bin/)
package.

```bash
git clone https://aur.archlinux.org/stitchmd-bin.git
cd stitchmd-bin
makepkg -si
```

If you use an AUR helper like [yay](https://github.com/Jguer/yay),
run the following command instead:

```go
yay -S stitchmd-bin
```

##### GitHub Releases

For **other platforms**, download a pre-built binary from the
[Releases page](https://github.com/abhinav/stitchmd/releases)
and place it on your `$PATH`.

#### Install from source

To install stitchmd from source, [install Go >= 1.20](https://go.dev/dl/)
and run:

```bash
go install go.abhg.dev/stitchmd@latest
```

### GitHub Action

[stitchmd-action](https://github.com/abhinav/stitchmd-action)
is a GitHub Action that will install and run stitchmd for you in CI.
With stitchmd-action, you can set up GitHub Workflows to:

- Validate that your output file is always up-to-date

  <details>

  ```yaml
  uses: abhinav/stitchmd-action@v1
  with:
    mode: check
    summary: doc/SUMMARY.md
    output: README.md
  ```

  </details>

- Automatically update your output file based on edits

  <details>

  ```yaml
  uses: abhinav/stitchmd-action@v1
  with:
    mode: write
    summary: doc/SUMMARY.md
    output: README.md

  # Optionally, use https://github.com/stefanzweifel/git-auto-commit-action
  # to automatically push these changes.
  ```

  </details>

- Install a binary of stitchmd and implement your own behavior

  <details>

  ```yaml
  uses: abhinav/stitchmd-action@v1
  with:
    mode: install
  ```

  </details>

For more information, see
[stitchmd-action](https://github.com/abhinav/stitchmd-action).

## Usage

```
stitchmd [OPTIONS] FILE
```

stitchmd accepts a single Markdown file as input.
This file defines the layout you want in your combined document,
and is referred to as the **summary file**.

For example:

```markdown
# User Guide

- [Getting Started](getting-started.md)
    - [Installation](installation.md)
- [Usage](usage.md)
- [API](api.md)

# Appendix

- [How things work](implementation.md)
- [FAQ](faq.md)
```

> The format of the summary file is specified in more detail in [Syntax](#syntax).

Given such a file as input, stitchmd will print a single Markdown file
including the contents of all listed files inline.

<details>
<summary>Example output</summary>

The output of the input file above
will be roughly in the following shape:

```markdown
# User Guide

- [Getting Started](#getting-started)
    - [Installation](#installation)
- [Usage](#usage)
- [API](#api)

## Getting Started

<!-- contents of getting-started.md -->

### Installation

<!-- contents of installation.md -->

## Usage

<!-- contents of usage.md -->

## API

<!-- contents of api.md -->

# Appendix

- [How things work](#how-things-work)
- [FAQ](#faq)

## How things work

<!-- contents of implementation.md -->

## FAQ

<!-- contents of faq.md -->
```

</details>

### Options

stitchmd supports the following options:

- [`-preface FILE`](#add-a-preface)
- [`-offset N`](#offset-heading-levels)
- [`-no-toc`](#disable-the-toc)
- [`-o FILE`](#write-to-file)
- [`-C DIR`](#change-the-directory)
- [`-d`](#report-a-diff)

#### Read from stdin

Instead of reading from a specific file on-disk,
you can pass in '-' as the file name to read the summary from stdin.

```bash
cat summary.md | stitchmd -
```

#### Add a preface

```
-preface FILE
```

If this flag is specified, stitchmd will include the given file
at the top of the output verbatim.

You can use this to add comments holding license headers
or instructions for contributors.

For example:

```bash
cat > generated.txt <<EOF
<!-- This file was generated by stitchmd. DO NOT EDIT. -->

EOF
stitchmd -preface generated.txt summary.md
```

#### Offset heading levels

```
-offset N
```

stitchmd changes heading levels based on a few factors:

- level of the section heading
- position of the file in the hierarchy of that section
- the file's own title heading

The `-offset` flag allows you to offset all these headings
by a fixed value.

<details>
<summary>Example</summary>

**Input**

```markdown
# User Guide

- [Introduction](intro.md)
  - [Installation](install.md)
```

```bash
stitchmd -offset 1 summary.md
```

**Output**

```markdown
## User Guide

- [Introduction](#introduction)
  - [Installation](#installation)

### Introduction

<!-- ... -->

### Installation

<!-- ... -->
```

</details>

Use a negative value to reduce heading levels.

<details>
<summary>Example</summary>

**Input**

```markdown
# User Guide

- [Introduction](intro.md)
  - [Installation](install.md)
```

```bash
stitchmd -offset -1 summary.md
```

**Output**

```markdown
# User Guide

- [Introduction](#introduction)
  - [Installation](#installation)

# Introduction

<!-- ... -->

## Installation

<!-- ... -->
```

</details>

#### Disable the TOC

```
-no-toc
```

stitchmd reproduces the original table of contents in the output.
You can change this with the `-no-toc` flag.

```bash
stitchmd -no-toc summary.md
```

This will omit the item listing under each section.

<details>
<summary>Example</summary>

**Input**

```markdown
- [Introduction](intro.md)
- [Installation](install.md)
```

```bash
stitchmd -no-toc summary.md
```

**Output**

```markdown
# Introduction

<!-- .. -->

# Installation

<!-- .. -->
```

</details>

#### Write to file

```
-o FILE
```

stitchmd writes its output to stdout by default.
Use the `-o` option to write to a file instead.

```bash
stitchmd -o README.md summary.md
```

#### Change the directory

```
-C DIR
```

Paths in the summary file are considered
**relative to the summary file**.

Use the `-C` flag to change the directory
that stitchmd considers itself to be in.

```bash
stitchmd -C docs summary.md
```

This is especially useful if your summary file is
[passed via stdin](#read-from-stdin)

```bash
... | stitchmd -C docs -
```

#### Report a diff

```
-d
```

stitchmd normally writes output directly to the file
if you pass in a filename with [`-o`](#write-to-file).
Use the `-d` flag to instead have it report what would change
in the output file without actually changing it.

```bash
stitchmd -d -o README.md # ...
```

This can be useful for lint checks and similar,
or to do a dry run and find out what would change
without changing it.

### Syntax

Although the summary file is Markdown,
stitchmd expects it in a very specific format.

The summary file is comprised of one or more **sections**.
Sections have a **section title** specified by a Markdown heading.

<details>
<summary>Example</summary>

```markdown
# Section 1

<!-- contents of section 1 -->

# Section 2

<!-- contents of section 2 -->
```

</details>

If there's only one section, the section title may be omitted.

```
File = Section | (SectionTitle Section)+
```

Each section contains a Markdown list defining one or more **list items**.
List items are one of the following,
and may optionally have another list nested inside them
to indicate a hierarchy.

- **Links** to local Markdown files:
  These files will be included into the output,
  with their contents adjusted to match their place.

    <details>
    <summary>Example</summary>

  ```markdown
  - [Overview](overview.md)
  - [Getting Started](start/install.md)
  ```
    </details>

- **Inclusions** of other summary files:
  These are links in the form `![title](file.md)`.
  The included file will be read as another summary file,
  and its sections will nested under this heading.

    <details>
    <summary>Example</summary>

  ```markdown
  - ![FAQ](faq.md)
  ```
    </details>

- **Plain text**:
  These will become standalone headers in the output.
  These **must** have a nested list.

    <details>
    <summary>Example</summary>

  ```markdown
  - Introduction
      - [Overview](overview.md)
      - [Getting Started](start/install.md)
  ```
    </details>

- **External links**:
  These will be written in the generated table-of-contents verbatim.
  They cannot have other items nested inside them.

    <details>
    <summary>Example</summary>

  ```markdown
  - [Overview](overview.md)
  - [Community](https://example.com)
  ```
    </details>

Items listed in a section are rendered together under that section.
A section is rendered in its entirety
before the listing for the next section begins.

<details>
<summary>Example</summary>

**Input**

```markdown
# Section 1

- [Item 1](item-1.md)
- [Item 2](item-2.md)

# Section 2

- [Item 3](item-3.md)
- [Item 4](item-4.md)
```

**Output**

```markdown
# Section 1

- [Item 1](#item-1)
- [Item 2](#item-2)

## Item 1

<!-- ... -->

## Item 2

<!-- ... -->

# Section 2

- [Item 3](#item-3)
- [Item 4](#item-4)

## Item 3

<!-- ... -->

## Item 4

<!-- ... -->
```

</details>

The heading level of a section determines the minimum heading level
for included documents: one plus the section level.

<details>
<summary>Example</summary>

**Input**

```markdown
## User Guide

- [Introduction](intro.md)
```

**Output**

```markdown
## User Guide

- [Introduction](#introduction)

### Introduction

<!-- ... -->
```

</details>

## Advanced

Advanced simulation using Gazebo11 engine with Ardupilot STIL via MAVROS communication protocol.

Please refer to the following repositories for further details.
- [Intelligent-Quads/iq_sim](https://github.com/Intelligent-Quads/iq_sim)  
- [monemati/multiuav-gazebo-simulation](https://github.com/monemati/multiuav-gazebo-simulation) 

### Ardupilot Installation

This repository uses the latest verion of Ardupilot as of time of writting (Arducopter V4.5.7 and Copter-4.5.7)

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install git

git clone --recurse-submodules https://github.com/your-github-userid/ardupilot
git clone https://github.com/ArduPilot/ardupilot.git

ls
cd ardupilot/
Tools/environment_install/install-prereqs-ubuntu.sh -y
mavproxy.py --version
pip show MAVProxy
. ~/.profile


git checkout Copter-4.5.7
git submodule update --init --recursive
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

### Gazebo Installation

This repository has been tested using Gazebo Classic. For Gazebo Ignition users, please refer to **Open Source Robotics Foundation** offical documention for ROS/Gazebo stable releases. 

#### Gazebo Classic Installation
```bash
cd
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
cat /etc/apt/sources.list.d/gazebo-stable.list
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo11
sudo apt-get install libgazebo11-dev
gazebo
```

#### Ardupilot Gazebo Plugin & Models
```bash
git clone https://github.com/SwiftGust/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install

sudo gedit ~/.bashrc
source /usr/share/gazebo/setup.sh

export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
source ~/.bashrc
```

### ROS Installation 
```bash
sudo apt-get update
sudo apt-get upgrade

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-get install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

#### Setup Catkin Workspace
```bash
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
pip3 install osrf-pycommon

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

### MAVROS installation 
Install MAVROS and MAVLink from source
```bash
cd ~/catkin_ws
wstool init ~/catkin_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

### IQ Simulation ROS package
```bash
cd ~/catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_sim.git

echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc
source ~/.bashrc

cd ~/catkin_ws
catkin build
source ~/.bashrc
roslaunch iq_sim runway.launch
```

### MAVProxy installation
```bash
https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html

sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
python3 -m pip install PyYAML mavproxy --user
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
source ~/.bashrc
```
## License

This software is distributed under the GPL-2.0 License:

```
stitchmd
Copyright (C) 2023 Abhinav Gupta

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
```