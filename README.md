# MissionPlanner_SwarmControl

![Dot Net](https://github.com/ardupilot/missionplanner/actions/workflows/main.yml/badge.svg) ![Android](https://github.com/ardupilot/missionplanner/actions/workflows/android.yml/badge.svg) ![OSX/IOS](https://github.com/ardupilot/missionplanner/actions/workflows/mac.yml/badge.svg)

## How to compile

### On Windows (Recommended)

#### 1. Install software

### Visual Studio Community
To compile Mission Planner, we recommend using Visual Studio. You can download Visual Studio Community from the [Visual Studio Download page](https://visualstudio.microsoft.com/downloads/ "Visual Studio Download page").

Visual Studio is a comprehensive suite with built-in Git support, but it can be overwhelming due to its complexity. To streamline the installation process, you can customize your installation by selecting the relevant "Workloads" and "Individual components" based on your software development needs.

To simplify this selection process, we have provided a configuration file that specifies the components required for MissionPlanner development. Here's how you can use it:

1. Go to "More" in the Visual Studio installer.
2. Select "Import configuration."
3. Use the following file: [vs2022.vsconfig](https://raw.githubusercontent.com/ArduPilot/MissionPlanner/master/vs2022.vsconfig "vs2022.vsconfig").

By following these steps, you'll have the necessary components installed and ready for Mission Planner development.

###### VSCode
Currently VSCode with C# plugin is able to parse the code but cannot build.

---

#### 2. Get the code

#### Method 1 Install using IDE
If you get Visual Studio Community, you should be able to use Git from the IDE. 
Clone `https://github.com/C-PANATORN/MissionPlanner_SwarmControl.git` to get the full code.

Open a git bash terminal in the MissionPlanner directory and type, "git submodule update --init" to download all submodules

#### Method 2 Install using Git (Recommended)

For users unfamiliar with Visual Studio Community, it is recommended to manually install the code using Git
```bash
git clone --recurse-submodules https://github.com/C-PANATORN/MissionPlanner_SwarmControl.git
```
From the MissionPlanner directory download all submodules
```bash
git submodule init
```

#### Method 3
Alternatively, if you have installed MissionPlanner from the main branch, you can manually replace the ".csproj" and "formation.cs" files directly from this repository and compile the code. **Note: if you use this method you need to delete all cashe memory of your previous installation**

#### 3. Build

To build the code:
- Open MissionPlanner.sln with Visual Studio
- From the Build menu, select "Build MissionPlanner"

**Note: if you run into build issue, please make sure Visual Studio and Git Submodules are propperly configured**

### On other systems
Building Mission Planner on other systems isn't support currently.