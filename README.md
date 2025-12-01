# ROS 2 Developer Tools

A comprehensive VS Code extension for ROS 2 developers that streamlines workspace management, build automation, and code generation.

## Features

### 🚀 Workspace Management
- **Initialize Workspace**: Quickly set up a ROS 2 workspace structure
- **Build Workspace**: Run `colcon build` with support for:
  - `--symlink-install` flag
  - `--packages-select` for selective builds
- **Clean Build**: Remove build artifacts (build/, install/, log/)

### 🎨 User Interface
- **Welcome Panel**: Beautiful webview with quick access to common commands
- **Sidebar Control Panel**: TreeView in the activity bar for easy navigation

### 📦 Package Generation
Create ROS 2 packages with a guided wizard:
- Support for both **ament_cmake** (C++) and **ament_python** packages
- Automatic generation of:
  - `package.xml` with proper dependencies
  - `CMakeLists.txt` for C++ packages
  - `setup.py` and `setup.cfg` for Python packages
  - Proper directory structure (src/, include/)

### 📝 Node Generation
Create ROS 2 nodes with automatic boilerplate:
- **C++ Nodes**:
  - Complete node skeleton with timer example
  - Automatic `CMakeLists.txt` updates
  - Proper executable and install configuration
- **Python Nodes**:
  - Complete node skeleton with timer example
  - Automatic `setup.py` entry point updates
  - Executable permissions set automatically

## Commands

Access these commands via the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`):

- `ROS 2: Initialize Workspace` - Create workspace structure
- `ROS 2: Build Workspace` - Build all packages
- `ROS 2: Clean Build` - Remove build artifacts
- `ROS 2: Open Panel` - Open the welcome panel
- `ROS 2: Create Package` - Launch package creation wizard
- `ROS 2: Create Node` - Launch node creation wizard

## Usage

### Creating a Package

1. Open Command Palette
2. Run `ROS 2: Create Package`
3. Follow the wizard:
   - Enter package name
   - Select build type (ament_cmake or ament_python)
   - Add dependencies (optional)
   - Add description (optional)

### Creating a Node

1. Open Command Palette
2. Run `ROS 2: Create Node`
3. Follow the wizard:
   - Select target package
   - Enter node name
   - Choose language (C++ or Python)

The extension will automatically:
- Generate the node file with boilerplate code
- Update CMakeLists.txt (C++) or setup.py (Python)
- Set proper file permissions

### Building Your Workspace

1. Open Command Palette
2. Run `ROS 2: Build Workspace`
3. View build output in the "ROS 2 Build" output channel

## Requirements

- VS Code 1.106.1 or higher
- ROS 2 installed on your system
- `colcon` build tool

## Extension Structure

```
src/
├── extension.ts              # Main entry point
├── views/
│   ├── welcomePanel.ts      # Webview panel
│   └── sidebarProvider.ts   # TreeView provider
├── services/
│   ├── commandRunner.ts     # Shell command executor
│   └── colconService.ts     # Colcon integration
├── wizards/
│   ├── packageWizard.ts     # Package creation wizard
│   └── nodeWizard.ts        # Node creation wizard
├── generators/
│   ├── packageGenerator.ts  # Package templates
│   └── nodeGenerator.ts     # Node templates
└── utils/
    └── workspaceUtils.ts    # Workspace utilities
```

## Development

### Building

```bash
npm install
npm run compile
```

### Testing

Press `F5` to launch the Extension Development Host.

### Packaging

```bash
npm install -g @vscode/vsce
vsce package
```

## License

TODO

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
