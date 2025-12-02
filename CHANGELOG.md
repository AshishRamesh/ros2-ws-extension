# Change Log

All notable changes to the "ROS 2 Developer Tools" extension will be documented in this file.

## [0.1.0] - 2025-12-02

### Added

#### Workspace Management
- **Build Integration**: Execute `colcon build` with interactive wizard
  - Package selection (all or specific packages)
  - Symlink install option
  - Output channel for build logs
- **Clean Build**: Remove build artifacts (build/, install/, log/ directories)
- **Package Creation Wizard**: Interactive package generation
  - Support for ament_cmake and ament_python
  - Dependency management
  - Automatic folder structure creation
- **Node Generation**: Create C++ and Python nodes
  - Skeleton code generation
  - Automatic CMakeLists.txt updates for C++ nodes
  - Automatic setup.py updates for Python nodes (ament_python)
  - Support for Python nodes in ament_cmake packages
- **.gitignore Generation**: Create ROS 2-specific .gitignore files

#### Run & Debug
- **Launch File Discovery**: Automatic scanning for .launch.py, .launch.xml, and .launch.yaml files
- **Launch File Execution**: Click-to-run launch files from sidebar
- **Node Discovery Service**: Scan CMakeLists.txt and setup.py for executable nodes
- **Run Configuration Wizard**: GUI-driven creation of launch.json configurations
  - Node selection
  - Custom environment sourcing
  - Integration with VS Code Run & Debug
- **Debug Adapter**: Native debugging support for ROS 2
  - Support for `ros2 launch` (launch files)
  - Support for `ros2 run` (individual nodes)
  - Custom source file configuration
  - Standardized on `request: "launch"` with `target` property

#### ROS 2 Topics
- **Topics Service**: List and query ROS 2 topics
  - Get topic type
  - Get publisher/subscriber counts
- **Echo Service**: Real-time topic message streaming
  - Frequency calculation
  - Message counting
  - Timestamp tracking
- **Topics Tree View**: Expandable topic list in sidebar
  - Refresh button in view title
  - Metadata display (type, publishers, subscribers, frequency, messages)
  - Click-to-view messages
- **Topic Echo Panel**: Dedicated webview for message display
  - Live message updates (last message only)
  - Timestamp display
  - Frequency and count statistics
  - Clear messages button

#### ROS 2 Bag
- **Bag Recording Wizard**: Interactive bag file recording
  - Custom file naming with validation
  - Directory selection via file picker
  - Topic selection (all topics or specific topics)
  - Terminal execution with `ros2 bag record`
- **Bag Playback Wizard**: Interactive bag file playback
  - Bag directory selection
  - Adjustable playback rate
  - Loop mode option
  - Additional options: start paused, clock publishing, keyboard controls
  - Terminal execution with `ros2 bag play`

#### User Interface
- **Organized Sidebar**: Four independent collapsible sections
  - Workspace: Build, clean, package/node creation, gitignore
  - Run & Debug: Launch files and nodes with run configuration
  - ROS 2 Topics: Topic listing and live echo
  - ROS 2 Bag: Recording and playback
- **Welcome Panel**: Quick access dashboard
  - ROS distro detection and display
  - Quick action buttons
  - Workspace status indication
- **Theme Icons**: Replaced all emoji icons with VS Code ThemeIcons for consistency
- **Integrated Terminals**: All ROS 2 commands execute in VS Code terminals

#### Services & Utilities
- **Environment Service**: ROS 2 environment detection and setup
  - Detect ROS_DISTRO
  - Validate workspace structure
  - Source setup scripts in terminals
- **Colcon Service**: Build system integration
  - Build with options
  - Clean workspace
  - Package selection
- **Command Runner**: Generic shell command execution utility
- **Workspace Utils**: Helper functions for workspace detection and validation

### Fixed
- Python node creation in ament_cmake packages (now updates CMakeLists.txt instead of setup.py)
- Debug configuration request type (standardized to `"launch"` with `target` property)
- Debug adapter validation logic to correctly handle both launch files and nodes
- Removed unused view providers and cleaned up imports

### Changed
- Refactored sidebar from single tree view to four separate view sections
- Moved refresh button from tree item to Topics view title bar
- Updated topic echo to show only last message instead of appending messages
- Consolidated launch files and nodes under "Run & Debug" section

### Technical Details
- TypeScript compilation with strict type checking
- VS Code API version: 1.106.1
- Node.js child_process for ROS 2 command execution
- Webview API for custom panels (Welcome Panel, Topic Echo Panel)
- TreeDataProvider API for sidebar views
- Debug Adapter Protocol for debugging integration

## [Unreleased]

### Planned Features
- ROS 2 parameter management
- Service call interface
- Action client interface
- TF tree visualization
- Node graph visualization
- Custom build profiles
- Workspace templates

---

**Note**: This is the initial release of ROS 2 Developer Tools. Future versions will include additional features and improvements based on user feedback.