# Change Log

All notable changes to the "ROS 2 Developer Tools" extension will be documented in this file.

## [0.2.0-alpha] - 2025-12-03

### Added
- **ROS 2 Workflows**: Introduced a powerful new system to automate complex development sequences
  - **Multi-step Automation**: Chain together launch files, nodes, shell commands, and delays
  - **Configuration Panel**: Visual interface to create, edit, and reorder workflow steps
  - **Sequential Execution**: Reliable execution of dependent tasks
  - **Terminal Reuse**: Option to run steps in the same terminal to keep context and reduce clutter
- **UI Polish**: Replaced emojis with professional SVG icons in Workflow Configuration Panel
  - Cleaner, more consistent look with VS Code design

### Fixed
- **Terminal Management Bug**: Resolved issue where "Run in previous terminal" would still create a new terminal
- **Code Cleanup**: Removed debug logging and unused code for better performance

### Improved
- **Codebase Optimization**: Comprehensive cleanup of `extension.ts` and services
  - Removed `console.log` statements
  - Removed unused imports and stale TODOs

## [0.1.1-alpha] - 2025-12-02

### Added
- **Enhanced Node Detection**: Improved CMakeLists.txt parsing to detect all C++ nodes
  - Now scans `add_executable()` directives
  - Scans `install(TARGETS ...)` directives for installed executables
  - Scans `install(PROGRAMS ...)` for Python scripts in CMake packages
  - Removes comments before parsing to avoid false matches
  - Validates node names (alphanumeric + underscore only)
  - Filters out CMake variables and keywords
- **Direct Node Execution**: Click any node to run it immediately
  - Auto-sources workspace `install/setup.bash`
  - Executes `ros2 run <package> <node>` in dedicated terminal
  - Works exactly like launch files (no configuration needed)
- **Terminal Reuse**: Smart terminal management for nodes and launch files
  - Reuses existing terminals by name instead of creating duplicates
  - Prevents terminal clutter from repeated runs
  - Preserves terminal history across runs
- **Auto-Refresh After Build**: Nodes and launch files automatically refresh when build completes
  - Callback system in `ColconService`
  - Ensures newly built nodes appear immediately

### Changed
- **Node Click Behavior**: Changed from opening Run Configuration Wizard to direct execution
- **VS Code Version Requirement**: Updated to ^1.104.1

### Fixed
- **Node Discovery**: Fixed recursive directory scanning to find all packages
  - Previously only found first package in nested directories
  - Now correctly scans all packages at any nesting level
- **CMakeLists.txt Parsing**: Improved regex patterns for multiline declarations
- **Terminal Management**: Prevents creation of duplicate terminals for same node/launch file

### Improved
- **Code Optimization**: Removed unused variables and properties
  - Removed unused `path` property from `DiscoveredNode` interface
  - Cleaned up unnecessary code
- **Logging**: Added comprehensive console logging for debugging
  - Shows which nodes are detected from each package
  - Logs parsing progress and results

### Technical Details
- Enhanced CMakeLists.txt scanning with 3 detection methods
- Better handling of CMake syntax variations
- Improved terminal lifecycle management
- Optimized node discovery performance

## [0.1.0-alpha] - 2025-12-02

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