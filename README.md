# ROS 2 Developer Tools

A comprehensive VS Code extension for ROS 2 developers, providing workspace management, build automation, code generation, debugging, and real-time topic monitoring.

> **If you find this extension useful, please consider giving it a star on GitHub! ⭐**

<div align="center">

[![GitHub stars](https://img.shields.io/github/stars/AshishRamesh/ros2-ws-extension?style=social)](https://github.com/AshishRamesh/ros2-ws-extension/stargazers)
[![GitHub views](https://komarev.com/ghpvc/?username=AshishRamesh-ros2-ws-extension&label=Views&style=flat-square)](https://github.com/AshishRamesh/ros2-ws-extension)

</div>


## Features

### 🔄 ROS 2 Workflows (New!)
- **Automated Sequences**: Chain together multiple tasks (launch files, nodes, commands, delays)
- **Visual Editor**: Drag-and-drop interface to configure and reorder steps
- **Terminal Reuse**: Run dependent steps in the same terminal to maintain context
- **One-Click Execution**: Run complex startup sequences with a single click

### 🏗️ Workspace Management
- **Build System Integration**: Execute `colcon build` with customizable options
- **Clean Build**: Remove build artifacts (build/, install/, log/)
- **Package Creation**: Interactive wizard for creating ROS 2 packages (ament_cmake/ament_python)
- **Node Generation**: Generate C++ and Python node skeletons with automatic build file updates
- **.gitignore Generation**: Create ROS 2-specific .gitignore files

### 🚀 Run & Debug
- **Launch File Discovery**: Automatically find and list all launch files in your workspace
- **Node Discovery**: Scan workspace for executable nodes (C++ and Python)
- **Run Configuration Wizard**: GUI-driven creation of `launch.json` configurations
- **Debug Integration**: Native VS Code debugging support for `ros2 launch` and `ros2 run`
- **Custom Environment Sourcing**: Configure custom setup scripts for each configuration

### 📡 ROS 2 Topics
- **Topic Listing**: View all available ROS 2 topics with refresh capability
- **Live Metadata**: Display topic type, publishers, subscribers, frequency, and message count
- **Topic Echo**: Real-time message viewing in a dedicated panel
- **Expandable Topics**: Click to view detailed information inline

### 📦 ROS 2 Bag
- **Record Bags**: Interactive wizard to record bag files
  - Custom file naming
  - Directory selection
  - Topic selection (all or specific topics)
- **Play Bags**: Playback wizard with advanced options
  - Adjustable playback rate
  - Loop mode
  - Start paused, clock publishing, keyboard control options

### 🎨 User Interface
- **Organized Sidebar**: Four collapsible sections (Workspace, Run & Debug, Topics, Bag)
- **Welcome Panel**: Quick access to common tasks
- **Integrated Terminals**: All ROS 2 commands run in VS Code terminals
- **Theme Icons**: Consistent with VS Code's native look and feel

## Installation

1. Open VS Code
2. Go to Extensions (Ctrl+Shift+X / Cmd+Shift+X)
3. Search for "ROS 2 Developer Tools"
4. Click Install

## Usage

### Getting Started

1. Open a ROS 2 workspace in VS Code
2. Click the ROS 2 icon in the Activity Bar (left sidebar)
3. Explore the sections: Workspace, Run & Debug, ROS 2 Topics, ROS 2 Bag, and Workflows

### Automating with Workflows

1. Expand "Workflows" section
2. Click "Configure Workflows"
3. Create a new workflow (e.g., "Full System Startup")
4. Add steps:
   - **Source**: Source your workspace
   - **Launch**: Select package and launch file
   - **Run**: Select package and node
   - **Command**: Run any shell command
   - **Delay**: Add wait time between steps
5. Check "Run in previous terminal" to chain commands in one terminal
6. Save and run from the sidebar!

### Building Your Workspace

**Option 1: Quick Build**
- Click "Build" in the Workspace section

**Option 2: Custom Build**
- Use Command Palette (Ctrl+Shift+P / Cmd+Shift+P)
- Run "ROS 2: Build Workspace"
- Select packages and options

### Creating Packages and Nodes

**Create a Package:**
1. Click "Create Package" in the Workspace section
2. Enter package name
3. Select build type (ament_cmake or ament_python)
4. Add dependencies (optional)

**Create a Node:**
1. Click "Create Node" in the Workspace section
2. Select package
3. Enter node name
4. Choose language (C++ or Python)

### Running Launch Files

1. Expand "Run & Debug" → "Launch Files"
2. Click on any launch file to run it

### Creating Run Configurations

1. Expand "Run & Debug" → "Nodes"
2. Click on a node
3. Configure source script (default: workspace setup.bash)
4. Configuration is added to `.vscode/launch.json`

### Monitoring Topics

1. Expand "ROS 2 Topics"
2. Click refresh icon (top-right) to update topic list
3. Expand any topic to view metadata
4. Click "Click here to view messages" to open live echo panel

### Recording and Playing Bags

**Record:**
1. Click "Record New Bag" in ROS 2 Bag section
2. Enter bag name
3. Select output directory
4. Choose topics (all or specific)

**Play:**
1. Click "Play Bag" in ROS 2 Bag section
2. Select bag directory
3. Set playback rate (e.g., 1.0x, 2.0x)
4. Choose loop mode
5. Configure additional options

## Requirements

- VS Code 1.104.0 or higher
- ROS 2 (Humble, Iron, Jazzy, or Rolling)
- colcon build tools
- Node.js and npm (for development)

## Extension Settings

This extension contributes the following settings:

* Currently no configurable settings (all features work out-of-the-box)

## Known Issues

- None currently reported

## Release Notes

See [CHANGELOG.md](CHANGELOG.md) for detailed release notes.

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## License

This extension is licensed under the MIT License.

## Support

For issues, questions, or feature requests, please visit the [GitHub repository](https://github.com/AshishRamesh/ros2-ws-extension).

---

**Enjoy developing with ROS 2!** 🤖
