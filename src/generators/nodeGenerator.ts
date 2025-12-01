import * as fs from 'fs';
import * as path from 'path';

export class NodeGenerator {
    /**
     * Generate a ROS 2 node
     */
    async generateNode(
        workspaceRoot: string,
        packageName: string,
        nodeName: string,
        language: 'cpp' | 'python'
    ): Promise<void> {
        const packagePath = path.join(workspaceRoot, 'src', packageName);

        if (!fs.existsSync(packagePath)) {
            throw new Error(`Package '${packageName}' does not exist`);
        }

        if (language === 'cpp') {
            await this.generateCppNode(packagePath, packageName, nodeName);
        } else {
            await this.generatePythonNode(packagePath, packageName, nodeName);
        }
    }

    private async generateCppNode(packagePath: string, packageName: string, nodeName: string): Promise<void> {
        const srcPath = path.join(packagePath, 'src');
        const nodeFilePath = path.join(srcPath, `${nodeName}.cpp`);

        // Check if file already exists
        if (fs.existsSync(nodeFilePath)) {
            throw new Error(`Node file '${nodeName}.cpp' already exists`);
        }

        // Generate C++ node skeleton
        const cppContent = `#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ${this.toPascalCase(nodeName)} : public rclcpp::Node
{
public:
  ${this.toPascalCase(nodeName)}()
  : Node("${nodeName}")
  {
    RCLCPP_INFO(this->get_logger(), "${nodeName} node has been started");
    
    // Create a timer that calls timer_callback every 1 second
    timer_ = this->create_wall_timer(
      1s, std::bind(&${this.toPascalCase(nodeName)}::timer_callback, this));
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<${this.toPascalCase(nodeName)}>());
  rclcpp::shutdown();
  return 0;
}
`;

        fs.writeFileSync(nodeFilePath, cppContent);

        // Update CMakeLists.txt
        await this.updateCMakeLists(packagePath, packageName, nodeName);
    }

    private async generatePythonNode(packagePath: string, packageName: string, nodeName: string): Promise<void> {
        const pythonPackagePath = path.join(packagePath, packageName);
        const nodeFilePath = path.join(pythonPackagePath, `${nodeName}.py`);

        // Check if file already exists
        if (fs.existsSync(nodeFilePath)) {
            throw new Error(`Node file '${nodeName}.py' already exists`);
        }

        // Generate Python node skeleton
        const pythonContent = `#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class ${this.toPascalCase(nodeName)}(Node):
    def __init__(self):
        super().__init__('${nodeName}')
        self.get_logger().info('${nodeName} node has been started')
        
        # Create a timer that calls timer_callback every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info('Timer callback triggered')


def main(args=None):
    rclpy.init(args=args)
    node = ${this.toPascalCase(nodeName)}()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
`;

        fs.writeFileSync(nodeFilePath, pythonContent);
        fs.chmodSync(nodeFilePath, '755'); // Make executable

        // Update setup.py
        await this.updateSetupPy(packagePath, packageName, nodeName);
    }

    private async updateCMakeLists(packagePath: string, packageName: string, nodeName: string): Promise<void> {
        const cmakeFilePath = path.join(packagePath, 'CMakeLists.txt');

        if (!fs.existsSync(cmakeFilePath)) {
            throw new Error('CMakeLists.txt not found');
        }

        let content = fs.readFileSync(cmakeFilePath, 'utf-8');

        // Find the position to insert the executable
        const addExecutableComment = '# Add executables here';
        const installComment = '# Install targets';

        if (content.includes(addExecutableComment)) {
            // Add executable after the comment
            const executableCode = `\nadd_executable(${nodeName} src/${nodeName}.cpp)\nament_target_dependencies(${nodeName} rclcpp)\n`;
            content = content.replace(addExecutableComment, addExecutableComment + executableCode);
        }

        if (content.includes(installComment)) {
            // Add install target
            const installCode = `\ninstall(TARGETS\n  ${nodeName}\n  DESTINATION lib/\${PROJECT_NAME}\n)\n`;
            content = content.replace(installComment, installComment + installCode);
        }

        fs.writeFileSync(cmakeFilePath, content);
    }

    private async updateSetupPy(packagePath: string, packageName: string, nodeName: string): Promise<void> {
        const setupPyPath = path.join(packagePath, 'setup.py');

        if (!fs.existsSync(setupPyPath)) {
            throw new Error('setup.py not found');
        }

        let content = fs.readFileSync(setupPyPath, 'utf-8');

        // Add entry point
        const entryPointComment = "# 'my_node = ";
        const entryPoint = `            '${nodeName} = ${packageName}.${nodeName}:main',\n`;

        if (content.includes("'console_scripts': [")) {
            // Find the console_scripts section and add the entry point
            const consoleScriptsRegex = /('console_scripts': \[)([\s\S]*?)(\s*\],)/;
            const match = content.match(consoleScriptsRegex);

            if (match) {
                const existingEntries = match[2].trim();
                let newContent;

                if (existingEntries && !existingEntries.startsWith('#')) {
                    // There are existing entries, add after them
                    newContent = `$1\n${existingEntries}\n${entryPoint}$3`;
                } else {
                    // No existing entries or only comments
                    newContent = `$1\n${entryPoint}$3`;
                }

                content = content.replace(consoleScriptsRegex, newContent);
            }
        }

        fs.writeFileSync(setupPyPath, content);
    }

    private toPascalCase(str: string): string {
        return str
            .split('_')
            .map(word => word.charAt(0).toUpperCase() + word.slice(1))
            .join('');
    }
}
